import itertools
import project
import re
import fsa
import logging
from LTLParser.LTLFormula import LTLFormula, LTLFormulaType
from createJTLVinput import createLTLfile
import specCompiler

class ExecutorResynthesisExtensions:
    """ Extensions to Executor to allow for specification rewriting and resynthesis.
        This class is not meant to be instantiated. """


    def getCurrentStateAsLTL(self, include_env=False):
        """ Return a boolean formula (as a string) capturing the current discrete state of the system (and, optionally, the environment as well) """

        if self.aut:
            # If we have current state in the automaton, use it (since it can capture
            # state of internal propositions).
            return fsa.stateToLTL(self.aut.current_state, include_env=include_env)
        else:
            # If we have no automaton yet, determine our state manually
            # TODO: support env
            # TODO: look at self.hsub.executing_config.initial_truths and pose
            return ""


    def _duplicateProject(self, proj, n=itertools.count(1)):
        """ Creates a copy of a proj, and creates an accompanying spec file with an 
            auto-incremented counter in the name.  (Not overwriting is mostly for debugging.)"""

        # reload from file instead of deepcopy because hsub stuff can include uncopyable thread locks, etc
        new_proj = project.Project()
        new_proj.setSilent(True)
        new_proj.loadProject(self.proj.getFilenamePrefix() + ".spec")

        # copy hsub references manually
        new_proj.hsub = proj.hsub
        new_proj.hsub.proj = new_proj # oh my god, guys
        new_proj.h_instance = proj.h_instance
        
        new_proj.sensor_handler = proj.sensor_handler
        new_proj.actuator_handler = proj.actuator_handler

        # Choose a name by incrementing the stepX suffix
        # Note: old files from previous executions will be overwritten
        base_name = self.proj.getFilenamePrefix().rsplit('.',1)[0] # without the modifier
        newSpecName = "%s.step%d.spec" % (base_name, n.next())

        new_proj.writeSpecFile(newSpecName)

        logging.info("Wrote new spec file: %s" % newSpecName)
        
        return new_proj

    def _setSpecificationInitialConditionsToCurrent(self, proj):
        """ Remove any existing initial conditions from the guarantees portion of the LTL specification
            and replace them with the current state of the system.

            Propositions that don't exist in both old and new specifications are ignored in the process."""

        # TODO: support doing this at the language level too?
        # TODO: what if state changes during resynthesis? should we be less restrictive?

        # parse the spec so we can manipulate it
        ltl_filename = proj.getFilenamePrefix() + ".ltl"
        assumptions, guarantees = LTLFormula.fromLTLFile(ltl_filename)

        # TODO: do we need to remove too? what about env?
        # add in current system state to make strategy smaller
        ltl_current_state = self.getCurrentStateAsLTL() # TODO: constrain to props in new spec
        gc = guarantees.getConjuncts()

        if ltl_current_state != "":
            gc.append(LTLFormula.fromString(ltl_current_state))

        # write the file back
        createLTLfile(ltl_filename, assumptions, gc)

    def resynthesizeFromNewSpecification(self, spec_text):
        self.pause()

        self.postEvent("INFO", "Starting resynthesis...")

        # Copy the current project
        new_proj = self._duplicateProject(self.proj)

        # Overwrite the specification text
        new_proj.specText = spec_text

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

        if not (realizable or realizableFS):
            logging.error("Specification for resynthesis was unsynthesizable!")
            self.pause()
            return False

        logging.info("New automaton has been created.")

        # Load in the new strategy

        self.proj = new_proj

        logging.info("Reinitializing execution...")

        spec_file = self.proj.getFilenamePrefix() + ".spec"
        aut_file = self.proj.getFilenamePrefix() + ".aut"
        self.initialize(spec_file, aut_file, firstRun=False)

        self.resume()

        return True
        
        # TODO: reload from file less often




