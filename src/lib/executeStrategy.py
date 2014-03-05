class ExecutorStrategyExtensions:
    """ Extensions to Executor to allow for the strategy structure (replacement of old FSA.py)."""

    def regionFromState(self, state):
        #TODO: maybe use domain
        """
        Given a state object, look at its 'bitX' outputs to determine the region encoded,
        and return the NUMBER of this region.
        """
        try:
            region = 0
            for bit in range(self.num_bits):
                if (int(state.outputs["bit" + str(bit)]) == 1):
                    # bit0 is MSB
                    region += int(2**(self.num_bits-bit-1))
        except KeyError:
            print "FATAL: Missing expected proposition 'bit%d' in automaton!" % bit
            region = None

        return region
        
    def findTransitionableStates(self, sensor_state):
        """
        Returns a list of states that we could conceivably transition to, given
        the environment state (determined by querying the sensor handler)

        If ``initial`` is true, the current region and output propositions will constrain
        state selection as well.
        
        sensor_state: current sensor proposition values in a dictionary
        """

        candidates = []

        # Define our pool of states to select from
        if initial:
            state_list = self.states
        else:
            state_list = self.current_state.transitions

        for state in state_list:
            okay = True

            if initial:
                if not self.proj.compile_options['fastslow']:
                    # First see if we can be in the state given our current region
                    if self.regionFromState(state) != self.current_region: continue
                else:
                    #TODO: with the new domain  --> replaced here for checking regions
                    pass
                    
                # Start only with Rank 0 states
                #if int(state.rank) != 0: continue

                # Now check whether our current output values match those of the state
                for key, value in state.outputs.iteritems():
                    # Ignore "bitX" output propositions
                    if re.match('^bit\d+$', key): continue

                    if int(self.current_outputs[key]) != int(value):
                        okay = False
                        break

                if not okay: continue

            # Now check whether our current sensor values match those of the state
            for key, value in state.inputs.iteritems():
                if int(sensor_state[key]) != int(value):
                    okay = False
                    break

            if okay:
                candidates.append(state)

        return candidates

    def runStrategyIteration(self):
        """
        Run, run, run the automaton!  (For one evaluation step)
        """

        # Let's try to transition
        next_states = self.findTransitionableStates()

        # Make sure we have somewhere to go
        if len(next_states) == 0:
            # Well darn!
            print "(FSA) ERROR: Could not find a suitable state to transition to!"
            return


        # See if we're beginning a new transition
        if next_states != self.last_next_states:
            # NOTE: The last_next_states comparison is also to make sure we don't
            # choose a different random next-state each time, in the case of multiple choices

            self.last_next_states = next_states

            # Only allow self-transitions if that is the only option!
            if len(next_states) > 1 and self.current_state in next_states:
                next_states.remove(self.current_state)

            self.next_state = random.choice(next_states)
            
            if not self.proj.compile_options['fastslow']:
                # normal execution
                self.next_region = self.regionFromState(self.next_state)
            else:
                #TODO: execution with fast slow
                pass  #self.next_region = self.regionFromState(self.next_state)

            print "Currently pursuing goal #{}".format(self.next_state.rank)

            # See what we, as the system, need to do to get to this new state
            self.transition_contains_motion = self.next_region is not None and (self.next_region != self.current_region)

            if self.proj.compile_options['fastslow']:
                # Run actuators before motion
                self.updateOutputs(self.current_state)

            if self.transition_contains_motion:
                # We're going to a new region
                print "Heading to region %s..." % self.regions[self.next_region].name

            self.arrived = False


        if not self.arrived:
            # Move one step towards the next region (or stay in the same region)
            self.arrived = self.motion_handler.gotoRegion(self.current_region, self.next_region)

        # Check for completion of motion
        if self.arrived and self.next_state != self.current_state:
            # TODO: Check to see whether actually inside next region that we expected

            if self.transition_contains_motion:
                print "Crossed border from %s to %s!" % (self.regions[self.current_region].name, self.regions[self.next_region].name)

            if not self.proj.compile_options['fastslow']:
                # Run actuators after motion
                self.updateOutputs(self.next_state)



            self.current_state = self.next_state
            self.current_region = self.next_region
            self.last_next_states = []  # reset
            print "Now in state %s (z = %s)" % (self.current_state.name, self.current_state.rank)
