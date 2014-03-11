#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import re
import strategy
import logging
import sys
import time
from collections import defaultdict

# TODO: make states hashable?
# TODO: .transitions should not be directly inside a state object because BDD

class FSAStrategy(strategy.Strategy):
    """
    An automaton object is a collection of state objects along with information about the
    current state of the automaton when being executed.
    """

    def __init__(self):
        # A collection of state objects belonging to the automaton
        self.states = strategy.StateCollection()
        self.transitions = defaultdict(lambda: defaultdict(bool)) # (state1, state2) -> T/F

    def configurePropositions(self, input_propositions, output_propositions,
                              input_domains, output_domains):

        self.states.clearPropositionsAndDomains()
        self.states.addInputPropositions(input_propositions)
        self.states.addOutputPropositions(output_propositions)
        for d in input_domains:
            self.states.addInputDomain(d)
        for d in output_domains:
            self.states.addOutputDomain(d)

    def loadFromFile(self, filename):
        """
        Create an automaton by reading in a file produced by a synthesizer,
        such as JTLV or Slugs.

        Basically just a lot of regexes.
        """

        tic = time.time()

        # Clear any existing states
        self.states.clearStates()

        # Initialize our state ID -> state object mapping, for fast lookup
        state_by_id = {}

        # Load the whole file into memory
        with open(filename, "r") as f:
            fsa_description = f.read()

        ###################
        # Read in states: #
        ###################

        # A magical regex to slurp up a state and its information all at once
        p = re.compile(r"State (?P<state_id>\d+) with rank (?P<goal_id>[\d\(\),-]+) -> <(?P<conds>(?:\w+:\d(?:, )?)+)>", re.IGNORECASE|re.MULTILINE)

        m = p.finditer(fsa_description)

        # Now, for each state we find:
        for match in m:
            # We'll add transitions later; first we have to create all the states so we can
            # refer to them when we define transitions

            # Get the State ID (at least the number that JTLV assigned the state; JTLV deletes states
            # during optimization, resulting in non-consecutive numbering which would be bad for binary
            # encoding efficiency, so we don't use these numbers internally except as state names)
            # and Goal ID ("rank" is a misnomer in the JTLV output; actually corresponds to index of currently pursued goal -- aka "jx").
            # This is the easy part.

            new_state = self.states.addNewState()
            new_state.state_id = match.group('state_id')
            new_state.goal_id = match.group('goal_id')

            # A regex so we can iterate over "PROP = VALUE" terms
            p2 = re.compile(r"(?P<var>\w+):(?P<val>\d)", re.IGNORECASE|re.MULTILINE)
            for prop_setting in p2.finditer(match.group('conds')):
                # Set the value of the proposition, casting string "0" or "1" to
                # appropriate boolean values
                prop_name, prop_value = prop_setting.groups()

                #### TEMPORARY HACK: REMOVE ME AFTER OTHER COMPONENTS ARE UPDATED!!!
                # Rewrite proposition names to make the old bitvector system work
                # with the new one
                prop_m = re.match('^bit(\d+)$', prop_name)
                if prop_m: prop_name = "region_b{}".format(prop_m.group(1))
                #################################################################

                if prop_value == "0":
                    new_state.setPropValue(prop_name, False)
                elif prop_value == "1":
                    new_state.setPropValue(prop_name, True)
                else:
                    raise ValueError("Proposition '{}' value of {!r} in state {} is invalid.".format(prop_name, prop_value, new_state.state_id))

            # Update mapping
            state_by_id[new_state.state_id] = new_state

        ########################
        # Read in transitions: #
        ########################

        # Another simple regex, this time for reading in transition definitions
        p = re.compile(r"State (?P<start>\d+)[^\n]+\s*With successors : (?P<ends>(?:\d+(?:, )?)+)", re.IGNORECASE|re.MULTILINE)
        m = p.finditer(fsa_description)

        # For each line:
        for match in m:
            # Get the state that the transitions come FROM
            start = match.group('start')
            # And make a list of the states that the transitions go TO
            ends = match.group('ends').split(', ')

            for end in ends:
                self.transitions[state_by_id[start]][state_by_id[end]] = True

        # All done, hooray!
        toc = time.time()
        logging.info("Loaded %d states in %f seconds.", len(self.states), toc-tic)


    # TODO: add a "getFirstState" to do this and make this return a list
    def searchForState(self, prop_assignments, state_list=None):
        """ Iterate through all known states (or a subset specified in `state_list`)
            and return the first one that matches `prop_assignments`.

            Returns None if no such state is found.  """

        if state_list is None:
            state_list = self.states

        satisfying_state = next((s for s in state_list if s.satisfies(prop_assignments)), None)

        return satisfying_state

    def findTransitionableStates(self, prop_assignments, from_state=None):
        if from_state is None:
            if self.current_state is None:
                raise ValueError("You must specify from_state if no current_state is set.")
            from_state = self.current_state

        transitionable_states = [s for s in self.transitions[from_state] if s.satisfies(prop_assignments)]

        return transitionable_states


def FSATest(spec_file_name):
    import project
    import pprint

    proj = project.Project()
    proj.loadProject(spec_file_name)
    aut_file_name = proj.getFilenamePrefix()+'.aut'
    s = FSAStrategy()
    s.loadFromFile(aut_file_name)
    region_domain = strategy.Domain("region", strategy.Domain.B0_IS_MSB, proj.rfi.regions)
    s.configurePropositions(proj.enabled_sensors, proj.enabled_actuators + proj.all_customs,
                            [], [region_domain])

    # TODO: should we be using region names instead of objects to avoid
    # weird errors if two copies of the same map are used?
    print "0th state:", s.states[0]

    initial_region = proj.rfi.regions[proj.rfi.indexOfRegionWithName("porch")]
    start_state = s.searchForState({"region": initial_region, "person": True})
    print "Start state:", start_state

    print "Successors:"
    pprint.pprint(s.findTransitionableStates({}, from_state=start_state))

if __name__ == "__main__":
    FSATest(sys.argv[1])
