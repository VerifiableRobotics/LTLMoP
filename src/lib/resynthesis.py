import itertools
import project
import re
import fsa
import logging
from LTLParser.LTLFormula import LTLFormula, LTLFormulaType
from createJTLVinput import createLTLfile
import specCompiler
import threading

class ExecutorResynthesisExtensions(object):
    """ Extensions to Executor to allow for specification rewriting and resynthesis.
        This class is not meant to be instantiated. """

    def __init__(self):
        super(ExecutorResynthesisExtensions, self).__init__()
        logging.info("Initializing resynthesis extensions...")

        # `next_proj` will store the temporary project we modify leading up to resynthesis
        self.next_proj = None

        # `needs_resynthesis` is an internal flag controlled by a resynthesis actuator handler
        # We don't call resynthesis code directly from the handler because we need to ensure
        # that it is only called after completion of all other actuators when transitioning to
        # a state that triggers resynthesis
        self.needs_resynthesis = False

        # Internal variables for receiving asynchronous user query responses
        self.user_query_response = ""
        self.received_user_query_response = threading.Event()

        ####################################################################################
        # HACK: Wrap the fsa runIteration function to check for internal flags at each step.
        # (Alternatively, this could be done with handler mapping manipulations or we could
        # put conditional code inside executor but this is least intrusive for now)
        ####################################################################################

        # We need to assign the functions to variables to make the
        # following closure work properly
        original_fsa_runiteration = fsa.Automaton.runIteration
        check_flags_func = self._checkForNewInternalFlags
        def runIterationWithResynthesisChecks(self, *args, **kwds):
            """ Check for internal flags after every FSA runIteration() call """
            original_fsa_runiteration(self, *args, **kwds)
            check_flags_func()

        # Update the FSA function to point to our new one
        fsa.Automaton.runIteration = runIterationWithResynthesisChecks

    def _checkForNewInternalFlags(self):
        """ Detect whether any "internal flags" (i.e. propositions beginning with an underscore)
            have become true in the last timestep, or resynthesis is pending; if so, perform an
            action based on this. """

        ### Look for rising, underscore-prefixed propositions ###

        # NOTE: This needs to be done /before/ resynthesis to ensure we are using the fully
        #       rewritten specification at resynthesis time

        # Check the current state of the automaton
        currently_true_props = set((p for p, v in self.aut.current_outputs.iteritems() if int(v) == 1))

        # Flags can't go true in the first time-step
        if not hasattr(self.aut, "previously_true_props"):
            self.aut.previously_true_props = currently_true_props
            return

        # Look for rising, underscore-prefixed propositions and process them
        rising_propositions = currently_true_props - self.aut.previously_true_props
        for p in rising_propositions:
            if p.startswith("_"):
                self._processInternalFlag(p)

        self.aut.previously_true_props = currently_true_props

        ### See if the resynthesis actuator handler has let us know we need to resynth ###

        if self.needs_resynthesis:
            if self.next_proj is not None:
                self.resynthesizeFromProject(self.next_proj)
            else:
                logging.error("Resynthesis was triggered before any spec rewrites.  Skipping.")

            # Clear the resynthesis flag
            self.needs_resynthesis = False

    def _processInternalFlag(self, flag_name):
        """ Respond appropriately to an "internal flag" proposition having been triggered.
            Note that this is only called on rising edges. """

        #################################################
        ### Check for group modification propositions ###
        #################################################

        # Use a regex on the proposition name to figure out what we should do
        m = re.match(r"_(?P<action>add_to|remove_from)_(?P<groupName>\w+)", \
                     flag_name, re.IGNORECASE)

        # We currently only handle this one type of flag, so there's nothing to do
        # if it doesn't match
        if m is None:
            return

        # Create next_proj if we haven't yet
        # This is what we'll be working on, and then eventually resynthesizing from
        if self.next_proj is None:
            self.next_proj = self._duplicateProject(self.proj)

        # We've been told to add or remove something from a group, but we need to figure
        # out exactly what that /something/ is (the "referent").  In some cases, this could probably
        # be resolved automatically using certain heuristics, but for now we will explicitly
        # ask the user what to do.

        # Keep asking the user until they give a non-empty response
        response = ""
        while response.strip() == "":
            response = self.queryUser("What should I add to the group {!r}?".format(m.group("groupName")))

        # Cast the referent to a list (there may be more than one in the case of new region detection)
        referents = [response]

        logging.debug("Resolved referents to {}.".format(referents))

        if m.group('action').lower() == "add_to":
            logging.info("Added item(s) %s to group %s.", ", ".join(referents), m.group('groupName'))

            # Rewrite the group definition in the specification text
            self._updateSpecGroup(m.group('groupName'), 'add', referents)

            # Get a list of region propositions
            region_names = [r.name for r in self.proj.rfi.regions]
            if self.proj.compile_options["decompose"]: # :((((
                region_names += [r.name for r in self.proj.rfiold.regions]

            # Add any new propositions to the project's proposition lists
            for ref in referents:
                if ref in self.next_proj.enabled_sensors + region_names:
                    # This prop already exists; don't need to add it.
                    continue

                # Assume all new user-added props are sensors for now.
                logging.debug("Adding new sensor proposition {!r}".format(ref))
                self.next_proj.enabled_sensors.append(ref)
                self.next_proj.all_sensors.append(ref)

            # TODO: update correlated groups (magically?)
            # TODO: add correlated group props (also magic?)

        elif m.group('action').lower() == "remove_from":
            # TODO: Removal from groups has not been tested and is likely not to work correctly

            # Rewrite the group definition in the specification text
            logging.info("Removed item(s) %s from group %s.", ", ".join(referents), m.group('groupName'))
            self._updateSpecGroup(m.group('groupName'), 'remove', referents)

    def _updateSpecGroup(self, group_name, operator, operand):
        """ Rewrite the text of the specification in `self.next_proj` by modifying proposition
            group `group_name` with operator `operator` (e.g. "add"/"remove") and
            operand `operand` (a list of propositions) """

        # Make a regex for finding the group definition in the spec
        PropositionGroupingRE = re.compile(r"^group\s+%s\s+(is|are)\s+(?P<propositions>.+)\n" % group_name, \
                                      re.IGNORECASE|re.MULTILINE)

        # Define a function for modifying the list of propositions in-place
        def gen_replacement_text(group_name, operand, m):
            """ Given a group name, modification operand, and match object from
                PropositionGroupingRE, return a new group definition line with an appropriately
                updated list of propositions."""

            # Figure out what propositions are already there
            propositions = re.split(r"\s*,\s*", m.group('propositions'))

            # Remove the "empty" placeholder if it exists
            propositions = [p for p in propositions if p != "empty"]

            # Perform the operation on the list of propositions
            if operator == "add":
                propositions.extend(operand)
            elif operator == "remove":
                propositions = [p for p in propositions if p not in operand]
            else:
                logging.error("Unknown group modification operator {!r}".format(operator))

            # Re-add the "empty" placeholder if the result of the operation is now empty
            if not propositions:
                propositions = ["empty"]

            # Create a new group definition line
            new_group_definition = "group %s is %s\n" % (group_name, ", ".join(propositions))

            return new_group_definition

        self.next_proj.specText = PropositionGroupingRE.sub(lambda m: gen_replacement_text(group_name, operand, m), \
                                                            self.next_proj.specText)

    def _duplicateProject(self, proj, n=itertools.count(1)):
        """ Creates a copy of a proj, and creates an accompanying spec file with an
            auto-incremented counter in the name."""

        # reload from file instead of deepcopy because hsub stuff can
        # include uncopyable thread locks, etc
        # TODO: fix this ^
        new_proj = project.Project()
        new_proj.setSilent(True)
        new_proj.loadProject(proj.getFilenamePrefix() + ".spec")

        # copy hsub references manually
        new_proj.hsub = proj.hsub
        new_proj.hsub.proj = new_proj # oh my god, guys
        new_proj.h_instance = proj.h_instance
        new_proj.rfiold = proj.rfiold

        new_proj.sensor_handler = proj.sensor_handler
        new_proj.actuator_handler = proj.actuator_handler

        # Choose a name by incrementing the stepX suffix
        # NOTE: old files from previous executions will be overwritten

        # Take the current proj name and remove any "stepX" part
        base_name = self.proj.getFilenamePrefix().rsplit('.', 1)[0]

        # Add a new "stepX" part
        newSpecName = "%s.step%d.spec" % (base_name, n.next())

        # Save the file
        new_proj.writeSpecFile(newSpecName)

        logging.info("Created new spec file: %s", newSpecName)

        return new_proj

    def _setSpecificationInitialConditionsToCurrent(self, proj):
        """ Remove any existing initial conditions from the guarantees portion of the
            LTL specification and replace them with the current state of the system.

            TODO: Propositions that don't exist in both old and new specifications are
            ignored in the process?"""

        # TODO: support doing this at the language level too?
        # TODO: what if state changes during resynthesis? should we be less restrictive?

        # Parse the LTL file in so we can manipulate it
        ltl_filename = proj.getFilenamePrefix() + ".ltl"
        assumptions, guarantees = LTLFormula.fromLTLFile(ltl_filename)

        # Get a conjunct expressing the current state
        ltl_current_state = self.getCurrentStateAsLTL() # TODO: Constrain to props in new spec
        logging.debug("Constraining new initial conditions to: " + ltl_current_state)

        # TODO: Do we need to remove pre-exisiting constraints too? What about env?
        # Add in current system state to make strategy smaller
        gc = guarantees.getConjuncts()
        if ltl_current_state != "":
            gc.append(LTLFormula.fromString(ltl_current_state))

        # Write the file back
        createLTLfile(ltl_filename, assumptions, gc)

    def resynthesizeFromProject(self, new_proj):
        """ Given a new project `new_proj`, pause execution, synthesize this new project,
            swap it in for the old project, and then resume execution. """

        # TODO: reload from file less often

        self.pause()

        self.postEvent("INFO", "Starting resynthesis. Please wait...")

        # Save the file
        new_proj.writeSpecFile()

        # Get a SpecCompiler ready
        c = specCompiler.SpecCompiler()
        c.proj = new_proj

        # Make sure rfi is non-decomposed here
        c.proj.loadRegionFile(decomposed=False)

        if c.proj.compile_options["decompose"]:
            c._decompose()

        # Call the parser
        c._writeLTLFile()
        c._writeSMVFile()

        # Constrain the initial conditions to our current state
        self._setSpecificationInitialConditionsToCurrent(new_proj)

        # Synthesize a strategy
        (realizable, realizableFS, output) = c._synthesize()
        logging.debug(output)

        # Check if synthesis succeeded
        if not (realizable or realizableFS):
            logging.error("Specification for resynthesis was unsynthesizable!")
            self.postEvent("INFO", "ERROR: Resynthesis failed.  Please check the terminal log for more information.")
            self.pause()
            return False

        logging.info("New automaton has been created.")

        # Load in the new strategy
        self.proj = new_proj

        logging.info("Reinitializing execution...")

        spec_file = self.proj.getFilenamePrefix() + ".spec"
        aut_file = self.proj.getFilenamePrefix() + ".aut"
        self.initialize(spec_file, aut_file, firstRun=False)

        # Clear next_proj again
        self.next_proj = None

        self.postEvent("INFO", "Resynthesis complete.  Resuming execution.")
        self.resume()

        return True

    def resynthesizeFromNewSpecification(self, spec_text):
        """ Given a text string of a new specification, resynthesize with a copy of the current
            project after swapping in the next `spec_text` for the previous specification. """

        # Copy the current project
        new_proj = self._duplicateProject(self.proj)

        # Overwrite the specification text
        new_proj.specText = spec_text
        
        return resynthesizeFromProject(new_proj)

    def getCurrentStateAsLTL(self, include_env=False):
        """ Return a boolean formula (as a string) capturing the current discrete state of 
            the system (and, optionally, the environment as well) """

        if self.aut:
            # If we have current state in the automaton, use it (since it can capture
            # state of internal propositions).
            return fsa.stateToLTL(self.aut.current_state, include_env=include_env)
        else:
            # If we have no automaton yet, determine our state manually
            # NOTE: This is only relevant for dialogue-based specifications
            # TODO: Support env
            # TODO: Look at self.proj.currentConfig.initial_truths and pose
            return ""

    def queryUser(self, question):
        """ Ask the user for an input. """
        # FIXME: This will have problems if a second query is issued before the first terminates

        # Delegate the query to whatever user interface is attached to execute (e.g. SimGUI)
        self.received_user_query_response.clear()
        self.postEvent("QUERY_USER", question)

        # Block until we receive a response
        # WARNING: The controller will be unresponsive during this period.
        self.received_user_query_response.wait()

        return self.user_query_response

    def processUserQueryResponse(self, answer):
        """ Callback function to receive a response to a user query. """

        logging.debug("Got user query response {!r}".format(answer))

        # Save the response
        self.user_query_response = answer

        # Let anyone waiting for the response know that it is ready
        self.received_user_query_response.set()
