#!/usr/bin/env python
# -*- coding: UTF-8 -*-

""" ==================================================================================================
    strategy.py - A Strategy object encodes a discrete strategy, which gives a system move in response
    to an environment move (or the reverse, in the case of a counterstrategy).
    ==================================================================================================
"""

import math
import re

# TODO: allow mopsy usage cleanly
# TODO: classmethod constructor that creates correct subclass based on filename
# TODO: clarify distinction between propositions as in bivalent t/f-only domains, propositions as raw
#       low-level prop names, and domains
# TODO: error if domain is evaluated and subprops are missing... uh oh

class Domain(object):
    """
    A Domain is a bit-vector abstraction, allowing a proposition to effectively have values other than
    just True and False.

    Domain "x" consists of propositions "x_b0", "x_b1", "x_b2", ..., and the value of the domain corresponds
    to the interpretation of these propositions as a binary string (following the order specified
    by `endianness`).  If `value_mapping` is specified, the numeric value of the domain will be used
    as an index into this array, and the corresponding element (must be non-integer) will be returned when the proposition value
    is queried, instead of a number (likewise, when setting the value of the proposition, this mapping will be
    used in reverse).

    `num_props` can be used to specify the size of the vector; if not specified,
    this will be automatically calculated based on the size of the `value_mapping` array.
    """

    B0_IS_MSB, B0_IS_LSB = range(2)

    def __init__(self, name, endianness=B0_IS_MSB, value_mapping=None, num_props=None):
        if not re.match(r"^[A-Za-z][A-Za-z0-9_]*$", name):
            raise ValueError("Name must begin with a letter and contain only alphanumeric characters or underscores.")

        self.name = name

        if endianness not in [Domain.B0_IS_MSB, Domain.B0_IS_LSB]:
            raise ValueError("Invalid endianness: choose either B0_IS_MSB or B0_IS_LSB.")
        self.endianness = endianness

        self.value_mapping = value_mapping
        if num_props is None:
            if value_mapping is None:
                raise TypeError("Cannot create domain without either value_mapping or num_props specified.")
            else:
                # Calculate the minimum number of bits necessary; note that we use max(1,...) because log(1)==0
                self.num_props = max(1, int(math.ceil(math.log(len(value_mapping),2))))
        else:
            self.num_props = num_props

    def propAssignmentsToValue(self, prop_assignments):
        """ Return the value of this domain, based on a dictionary [prop_name(str)->value(bool)] of
            the values of the propositions composing this domain.
        """
        n = self.propAssignmentsToNumericValue(prop_assignments)

        if self.value_mapping is None:
            return n
        else:
            return self.value_mapping[n]

    def propAssignmentsToNumericValue(self, prop_assignments):
        """ Convert a dictionary [prop_name(str)->value(bool)] of propositions composing this domain
            into an integer value.
        """

        value = 0
        for bit, prop_name in enumerate(self.getPropositions()):
            if prop_assignments[prop_name]:
                if self.endianness == Domain.B0_IS_MSB:
                    value += 2**((self.num_props-1)-bit)
                else:
                    value += 2**(bit-1)

        return value

    def valueToPropAssignments(self, value):
        """ Convert a value into the corresponding dictionary [prop_name(str)->value(bool)]
            of propositions composing this domain
        """

        if self.value_mapping is None:
            if isinstance(value, int):
                n = value
            else:
                raise TypeError("Non-integral values are not permitted without a value_mapping.")
        else:
            n = self.value_mapping.index(value)

        return self.numericValueToPropAssigments(n)

    def numericValueToPropAssigments(self, number):
        """ Convert an integer value into the corresponding dictionary [prop_name(str)->value(bool)]
            of propositions composing this domain
        """

        # Perform input sanity checks
        if not isinstance(number, int):
            raise TypeError("Cannot set domain to non-integral value.")

        if number < 0:
            raise TypeError("Cannot set domain to negative value.")

        # Convert to a left-padded bitstring
        bs = "{0:0>{1}}".format(bin(number)[2:], self.num_props)

        # Create a dictionary based on the bitstring
        return {"{}_b{}".format(self.name, bit):(v=="1") for bit, v in
                enumerate(bs if self.endianness == Domain.B0_IS_MSB else reversed(bs))}

    def getPropositions(self):
        """ Returns a list of the names of the propositions that are covered by this domain. """

        return ["{}_b{}".format(self.name, bit) for bit in range(self.num_props)]

    def __str__(self):
        return '<Domain "{0}" ({0}_b0:{0}_b{1})>'.format(self.name, self.num_props-1)

class State(object):
    """
    At its most basic, a state has only binary proposition values (this is
    stored internally as the set of propositions that are true in the state);
    more information can be attached by subclasses

    When created, a reference to the parent StateCollection needs to be be
    passed so that the state is aware of its evaluation context
    """

    def __init__(self, parent, prop_assignments=None):
        if not isinstance(parent, StateCollection):
            raise TypeError("The parent of a State must be a StateCollection.")

        self.context = parent
        self.true_props = set() # Set of names of propositions that are true in this state

        # TODO: formalize this part
        self.state_id = None
        self.goal_id = None

        if prop_assignments is not None:
            self.setPropValues(prop_assignments)

    def getInputs(self, eval_domains=True):
        """ Return a dictionary of input proposition values for this state.

            If `eval_domains` is True, ignore individual bit-vector propositions and return only the value of
            the relevant domain in its entirety. """

        if eval_domains:
            return self.getPropValues(self.context.collapseDomains(self.context.input_props))
        else:
            return self.getPropValues(self.context.input_props)

    def getOutputs(self, eval_domains=True):
        if eval_domains:
            return self.getPropValues(self.context.collapseDomains(self.context.output_props))
        else:
            return self.getPropValues(self.context.output_props)

    def getAll(self, eval_domains=True):
        all_props = self.context.input_props + self.context.output_props
        if eval_domains:
            return self.getPropValues(self.context.collapseDomains(all_props))
        else:
            return self.getPropValues(all_props)

    def satisfies(self, prop_assignments):
        """ Returns True iff the proposition settings in this state agree with all
            prop_assignments.  Any unspecified propositions are treated as don't-cares. """

        return all((self.getPropValue(k) == v for k, v in prop_assignments.iteritems()))

    def getPropValues(self, names):
        return {p:self.getPropValue(p) for p in names}

    def getDomainValue(self, name):
        d = self.context.getDomainByName(name)
        return d.propAssignmentsToValue(self.getAll(eval_domains=False))

    def getPropValue(self, name):
        try:
            return self.getDomainValue(name)
        except ValueError:
            return (name in self.true_props)

    def setPropValue(self, prop_name, prop_value):
        # TODO: warning if prop_assignments are non-sensical, or maybe check whether
        # we are leaving some values undefined (though this might even be OK in some circumstances?)

        # First, see if this prop_name is a domain
        try:
            domain = self.context.getDomainByName(prop_name)
        except ValueError:
            domain = None

        if domain is not None:
            # Handle domains
            # TODO: Are we being ridiculously inefficient here?  Should we stop using sets?
            #       Should we only deal with underlying bit-vectors on import/export instead of
            #       continuously mapping back and forth on every data structure access?
            for subprop_name, subprop_value in domain.valueToPropAssignments(prop_value).iteritems():
                self.setPropValue(subprop_name, subprop_value)
        else:
            # Handle boolean propositions
            if not isinstance(prop_value, bool):
                raise ValueError("Can only assign boolean values to non-Domain propositions.")

            if prop_value:
                self.true_props.add(prop_name)
            else:
                self.true_props.discard(prop_name)

    def setPropValues(self, prop_assignments):
        for prop_name, prop_value in prop_assignments.iteritems():
            self.setPropValue(prop_name, prop_value)

    ######## TODO: fix this function
    def stateToLTL(state, use_next=False, include_env=True, swap_io=False):
        """ swap_io is for the counterstrategy aut in mopsy """

        def decorate_prop(prop, polarity):
            if use_next:
                prop = "next({})".format(prop)
            if int(polarity) == 0:
                prop = "!"+prop
            return prop

        inputs = state.inputs
        outputs = state.outputs

        if swap_io:
            inputs, outputs = outputs, inputs

        sys_state = " & ".join([decorate_prop("s."+p, v) for p,v in outputs.iteritems()])

        if include_env:
            env_state = " & ".join([decorate_prop("e."+p, v) for p,v in inputs.iteritems()])
            return " & ".join([env_state, sys_state])
        else:
            return sys_state

    def __eq__(self, other):
        return (self.context is other.context) and (self.true_props == other.true_props)

    def __repr__(self):
        # TODO: print as LTL instead?
        return "<State with assignment: inputs = {}, outputs = {}>".format(self.getInputs(), self.getOutputs())

class StateCollection(list):
    """
    StateCollection is a simple extension of list, to allow for keeping track of
    meta-information about states, such as which propositions are inputs and which
    are outputs, as well as domains. (These are not class properties of State
    because different StateCollections might have different settings.)
    """

    def __init__(self, *args, **kwds):
        self.clearPropositionsAndDomains()
        super(StateCollection, self).__init__(*args, **kwds)

    def clearPropositionsAndDomains(self):
        self.input_props = []
        self.output_props = []
        self.domains = []

    def clearStates(self):
        del self[:]

    def addInputDomain(self, domain):
        self.domains.append(domain)
        self.input_props.extend(domain.getPropositions())

    def addOutputDomain(self, domain):
        self.domains.append(domain)
        self.output_props.extend(domain.getPropositions())

    def addInputPropositions(self, prop_list):
        # TODO: error on non-list input because strings are quietly accepted..
        self.input_props.extend(prop_list)

    def addOutputPropositions(self, prop_list):
        self.output_props.extend(prop_list)

    def addNewState(self, prop_values=None):
        new_state = State(self, prop_values)
        self.append(new_state)

        return new_state

    def getDomainOfProposition(self, prop_name):
        # TODO: There might be ways to make all these lookups more efficient
        return next((d for d in self.domains if prop_name in d.getPropositions()), None)

    def collapseDomains(self, prop_name_list):
        """ Given a list of proposition names, remove any names contained in a domain
            and replace them with the name of the relevant domain.

            Note that a given domain will be included in the output list only once,
            and will be included even if not all of its sub-propositions have been provided. """

        # TODO: this seems excessive
        new_list = []
        for prop_name in prop_name_list:
            d = self.getDomainOfProposition(prop_name)
            if d is not None:
                if d.name not in new_list:
                    new_list.append(d.name)
            else:
                new_list.append(prop_name)

        return new_list

    def getDomainByName(self, name):
        try:
            return next((d for d in self.domains if d.name == name))
        except StopIteration:
            raise ValueError("No domain defined with name '{}'".format(name))

class Strategy(object):
    def __init__(self):
        self.current_state = None

    def transitionExists(self, state1, state2):
        raise NotImplementedError("Use a subclass of Strategy")

    def iterateOverStates(self):
        raise NotImplementedError("Use a subclass of Strategy")

    def loadFromFile(self, filename):
        raise NotImplementedError("Use a subclass of Strategy")

    def searchForState(self, prop_assignments, state_list=None):
        raise NotImplementedError("Use a subclass of Strategy")

    def findTransitionableStates(self, prop_assignments, from_state=None):
        raise NotImplementedError("Use a subclass of Strategy")

    def dumpStates(self, range=None):
        """
        Print out the contents of the automaton in a human-readable format
        """
        if range is None:
            range = self.states

        for state in range:
            print "Name: ", state.name
            print "Inputs: "
            for key, val in state.inputs.iteritems():
                print key + " = " + val
            print "Outputs: "
            for key, val in state.outputs.iteritems():
                print key + " = " + val
            print "Transitions: "
            for trans in state.transitions:
                print trans.name

    # TODO: Fix these and maybe move to a different StrategyDumper file
    #def writeDot(self, filename):
        #"""
        #Write a dot file so we can look at the automaton visually.
        #"""

        #FILE = open(filename,"w")

        ## Write the header
        #FILE.write('digraph A { \n')
        #FILE.write('\trankdir=LR;\n')
        ##FILE.write('\tratio = 0.75;\n')
        #FILE.write('\tsize = "8.5,11";\n')
        #FILE.write('\toverlap = false;\n')
        ##FILE.write('\tlayout = hierarchical;\n')

        ## Write the states with region and outputs that are true
        #for state in self.states:
            #FILE.write('\ts'+ state.name + ' [style=\"bold\",width=0,height=0, fontsize = 20, label=\"')
            #stateRegion = self.regionFromState(state)
            #FILE.write( self.getAnnotatedRegionName(stateRegion) + '\\n')
            #for key in state.outputs.keys():
                #if re.match('^bit\d+$',key): continue
                #if state.outputs[key] == '1':
                    #FILE.write( key + '\\n')
                #else:
                    #FILE.write( '!' + key + '\\n')
            ##FILE.write( "("+state.rank + ')\\n ')
            #FILE.write('\" ];\n')

        ## Write the transitions with the input labels (only inputs that are true)
        #for state in self.states:
            #for nextState in state.transitions:
                #FILE.write('\ts'+ state.name +' -> s'+ nextState.name +'[style=\"bold\", arrowsize = 1.5, fontsize = 20, label=\"')
                ## Check the next state to figure out which inputs have to be on
                #envRegion = self.envRegionFromState(nextState)
                #if envRegion is not None:
                    #FILE.write( self.getAnnotatedRegionName(envRegion) + '\\n')
                #for key in nextState.inputs.keys():
                    #if re.match('^sbit\d+$',key): continue
                    #if nextState.inputs[key] == '1':
                        #FILE.write( key + '\\n')
                    #else:
                        #FILE.write( '!' + key + '\\n')
                #FILE.write('\" ];\n')

        #FILE.write('} \n')
        #FILE.close()

    #def writeSMV(self, filename):
        #"""
        #Write an SMV file so we can model check the synthesized automaton.
        #"""

        #FILE = open(filename,"w")

        ## Write the header
        #FILE.write('MODULE main\n')
        #FILE.write('\tVAR\n')
        #state1 = self.states[1]
        #for key, val in state1.inputs.iteritems():
            #FILE.write("\t\t"+key+": boolean;\n")
        #for key, val in state1.outputs.iteritems():
          #FILE.write("\t\t"+key+": boolean;\n")
        #FILE.write("\t\t"+"rank : 0..10;\n")
        #FILE.write("\tINIT\n")
        #FILE.close()


        ##Select the initial conditions (i.e. substitutions on the initial conditions
        ##grab all lines that do not have any temporal operators, i.e. the initial conditions.
        ##cmd = "grep -v '[\[;-]' "+self.proj.getFilenamePrefix()+".ltl | grep -v next | grep -v \($ | sed -e 's/s\.//g' -e 's/e\.//g' >> "+filename
        ##os.system(cmd)
        #input = open(self.proj.getFilenamePrefix()+".ltl")
        #output = open(filename, 'a')
        #for line in input:
            #if (re.search("[\[;-]", line) is None and re.search("next", line) is None and re.search("\($", line) is None):
                #line = re.sub("s\.", "", line)
                #line = re.sub("e\.", "", line)
                #output.write(line)
        #input.close()
        #output.close()

        ## Write the transitions as a disjunction of conjunctions
        #FILE = open(filename,"a")
        #FILE.write("TRUE\n")
        #FILE.write("\tTRANS\n")
        #for state in self.states:
            #for nextState in state.transitions:
                ##FILE.write('\t'+ "&".join( map( lambda x: str(x[0] + " = " + str(x[1])), state.inputs.items())) + "&".join( map( lambda x: str(x[0] + " = " + str(x[1])), state.outputs.items() ) ) + "|" )
                ## Check the next state to figure out which inputs have to be on
                ##The extra TRUE and FALSE clauses circumvent the need to account for trailing &s.
                #FILE.write(' ((')
                #for key in state.inputs.keys():
                    #if state.inputs[key] == '1':
                        #FILE.write( key + ' & ')
                    #else:
                        #FILE.write( '!' + key + ' & ')
                #for key in state.outputs.keys():
                    #if state.outputs[key] == '1':
                        #FILE.write( key + ' & ')
                    #else:
                        #FILE.write( '!' + key + ' & ')
                #FILE.write( "rank = " + state.rank + ' ) & ')
                #for key in nextState.inputs.keys():
                    #if nextState.inputs[key] == '1':
                        #FILE.write("next("+ key + ') & ')
                    #else:
                        #FILE.write( "! next("+ key + ') & ')
                #for key in nextState.outputs.keys():
                    #if nextState.outputs[key] == '1':
                        #FILE.write("next("+ key + ') & ')
                    #else:
                        #FILE.write( "! next("+ key + ') & ')
                #FILE.write( "next(rank) = " + nextState.rank + ' & ')
                #FILE.write(' TRUE) | ')
        #FILE.write('FALSE')
        #FILE.write('\n')
        #FILE.write("\tLTLSPEC\n")
        #FILE.close()

        ##replace next, <> and [] with X, F and G, and gets rid of the e. and s. prefixes. It also puts an implication between the two parts of the spec.
        ##cmd = "cat " + self.proj.getFilenamePrefix()+".ltl | sed -e 's/\[\]/G /g' -e 's/^<^>/F /g'  -e 's/next/X /g' -e 's/s\.//g' -e 's/e\.//g' -e '0,/;/s/;/-^>/' -e '/LTLSPEC/ d' -e '/--/d' >> " + filename
        ##os.system(cmd)
        #input = open(self.proj.getFilenamePrefix()+".ltl")
        #output = open(filename, 'a')
        #impFlag = False
        #for line in input:
            #if (re.search("LTLSPEC", line) is None and re.search("--", line) is None):
                #line = re.sub("\[\]", "G ", line)
                #line = re.sub("<>", "F ", line)
                #line = re.sub("next", "X ", line)
                #line = re.sub("s\.", "", line)
                #line = re.sub("e\.", "", line)
                #if not impFlag and not re.search(";", line) is None:
                    #line = re.sub(";", "->", line)
                    #impFlag = True
                #output.write(line)
        #input.close()
        #output.close()




        ## Write the transitions with the input labels (only inputs that are true)


if __name__ == "__main__":
    #### Test: Domains ####
    animals = ["cat", "dog", "red-backed fairywren", "pseudoscorpion", "midshipman"]
    d = Domain("favorite_animal", value_mapping=animals)
    print d
    assert d.num_props == 3
    for value in animals:
        p = d.valueToPropAssignments(value)
        print p
        assert value == d.propAssignmentsToValue(p)

    ### Test: StateCollections & States ###
    states = StateCollection()
    regions = ["kitchen", "living", "bedroom"]
    states.addOutputDomain(Domain("region", Domain.B0_IS_MSB, regions))
    states.addInputDomain(Domain("nearby_animal", Domain.B0_IS_LSB, animals))
    states.addInputPropositions(("low_battery",))
    states.addOutputPropositions(("hypothesize", "experiment", "give_up"))
    states.addNewState({"region": "bedroom", "low_battery": True})
    print "Inputs:", states.input_props
    print "Outputs:", states.output_props
    print "Domains:", states.domains
    for state in states:
        print state.true_props
        print state

