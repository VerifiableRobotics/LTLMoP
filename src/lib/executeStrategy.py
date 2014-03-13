import fsa
import sys
import logging,random
import project

class ExecutorStrategyExtensions(object):
    """ Extensions to Executor to allow for the strategy structure (replacement of old FSA.py)."""
    
    def __init__(self):
        super(ExecutorStrategyExtensions, self).__init__()
        logging.info("Initializing strategy extensions...")
        
        self.last_next_states= []
        self.next_state      = []
        self.current_region  = []
        self.next_region     = []        
        
    def updateOutputs(self, state=None):
        """
        Update the values of current outputs in our execution environment to reflect the output
        proposition values associated with the given state
        """

        if state is None:
            state = self.current_state

        for key, output_val in state.getOutputs().iteritems():
            # Skip any region 
            if 'region' == key: continue

            if key not in self.current_outputs.keys() or output_val != self.current_outputs[key]:

                # The state of this output proposition has changed!
                self.postEvent("INFO", "Output proposition \"%s\" is now %s!" % (key, str(output_val)))

                # Run any actuator handlers if appropriate
                if key in self.proj.enabled_actuators:
                    #TODO: figure out a way to save motion_handler after project is called
                    self.proj.h_instance['motionControl'].gotoRegion(self.current_region, self.current_region)  # Stop, in case actuation takes time
                    initial=False
                    new_val = output_val #taken by the actuator argument, has to be called new_val
                    exec(self.proj.actuator_handler[key])

                self.current_outputs[key] = output_val

    def runStrategyIteration(self):
        """
        Run, run, run the automaton!  (For one evaluation step)
        """
        #FIXME: motion control handler should change to taking region objects
        self.current_region = self.proj.rfi.regions.index(self.aut.current_state.getPropValue('region'))

        # Take a snapshot of our current sensor readings
        # This is so we don't risk the readings changing in the middle of our state search
        #TODO: need to fetch from handleSub
        self.sensor_handler = self.proj.sensor_handler
        self.h_instance = self.proj.h_instance
        sensor_state = {}
        for sensor in self.proj.enabled_sensors:
            sensor_state[sensor]  = eval(self.sensor_handler[sensor], {'self':self,'initial':False}) 
        
        # Let's try to transition 
        # TODO: set current state so that we don't need to call from_state
        next_states = self.aut.findTransitionableStates(sensor_state, from_state= self.aut.current_state)

        # Make sure we have somewhere to go
        if len(next_states) == 0:
            # Well darn!
            logging.error("Could not find a suitable state to transition to!")
            return

        # See if we're beginning a new transition
        if next_states != self.last_next_states:
            # NOTE: The last_next_states comparison is also to make sure we don't
            # choose a different random next-state each time, in the case of multiple choices

            self.last_next_states = next_states

            # Only allow self-transitions if that is the only option!
            if len(next_states) > 1 and self.aut.current_state in next_states:
                next_states.remove(self.aut.current_state)

            self.next_state = random.choice(next_states)
            self.next_region = self.proj.rfi.regions.index(self.next_state.getPropValue('region'))
                
            self.postEvent("INFO", "Currently pursuing goal #{}".format(self.next_state.goal_id))

            # See what we, as the system, need to do to get to this new state
            self.transition_contains_motion = self.next_region is not None and (self.next_region != self.current_region)

            if self.proj.compile_options['fastslow']:
                # Run actuators before motion
                self.updateOutputs(self.next_state)

            if self.transition_contains_motion:
                # We're going to a new region
                self.postEvent("INFO", "Heading to region %s..." % self.proj.rfi.regions[self.next_region].name)

            self.arrived = False

        if not self.arrived:
            #TODO: fix motion_handler is none for mospy and viewing automaton
            # Move one step towards the next region (or stay in the same region)
            self.arrived = self.proj.h_instance['motionControl'].gotoRegion(self.current_region, self.next_region)
         
        # Check for completion of motion
        if self.arrived and self.next_state != self.aut.current_state:
            # TODO: Check to see whether actually inside next region that we expected
            if self.transition_contains_motion:
                self.postEvent("INFO", "Crossed border from %s to %s!" % (self.proj.rfi.regions[self.current_region].name, self.proj.rfi.regions[self.next_region].name))

            if not self.proj.compile_options['fastslow']:
                # Run actuators after motion
                self.updateOutputs(self.next_state)

            self.aut.current_state = self.next_state
            self.current_region = self.next_region
            self.last_next_states = []  # reset
            
            self.postEvent("INFO", "Now in state %s (z = %s)" % (self.aut.current_state.state_id, self.aut.current_state.goal_id))
