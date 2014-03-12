#!/usr/bin/env python
# -*- coding: UTF-8 -*-

""" ==============================================================================
    strategy.py - A Strategy object encodes a discrete strategy, which gives a
    system move in response to an environment move (or the reverse, in the case of
    a counterstrategy).
    ==============================================================================
"""

import math
import re

# TODO: make sure this works with mopsy
# TODO: classmethod constructor that creates correct subclass based on filename

class Domain(object):
    """ A Domain is a bit-vector abstraction, allowing a proposition to effectively
    have values other than just True and False.

        Domain "x" consists of propositions "x_b0", "x_b1", "x_b2", ..., and the
    value of the domain corresponds to the interpretation of these propositions as
    a binary string (following the order specified by `endianness`).  If
    `value_mapping` is specified, the numeric value of the domain will be used as
    an index into this array, and the corresponding element (must be non-integer)
    will be returned when the proposition value is queried, instead of a number
    (likewise, when setting the value of the proposition, this mapping will be used
    in reverse).

        `num_props` can be used to specify the size of the vector; if not
    specified, this will be automatically calculated based on the size of the
    `value_mapping` array.
    """

    # TODO: add code for generating LTL mutexes
    # TODO: add non-bitvector mode (e.g. region_kitchen)-- but requires mutex!

    # Constants to indicate endianness options
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
            if prop_name not in prop_assignments:
                raise ValueError("Cannot evaluate domain {!r} because expected subproposition {!r} is undefined.".format(self.name, prop_name))

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
    A state, at its most basic, consists of a value assignment to propositions
    (represented as a dictionary {proposition name (string) -> proposition value}).

    Additional metadata can be attached as necessary.

    When created, a reference to the parent StateCollection needs to be be
    passed so that the state is aware of its evaluation context.

    A Note About Multi-Valent Propositions:

        Multivalent propositions (i.e. those whose value can span a Domain), are
    handled fairly flexibly internally, but most of the dynamic translation is
    hidden from the user.

        In general, read access to propositions will always be presented at the
    highest-level possible.  For example, when asking for the value of a state,
    multivalent propositions are presented instead of the underlying binary
    subpropositions (unless explicitly overridden by using the `expand_domains`
    flag provided by some functions).  That said, if one wishes to query the value
    of a subproposition for some reason, its value will be calculated
    automatically.

        In a similar vein, in order to minimize the worries of those using this
    module, multivalent propositions can be written to-- and are stored
    internally-- in one of two ways: either as multiple binary assignments to the
    subpropositions of the domain, or a single value assignment to the domain
    proposition itself.  This latter form is preferred, since it is simplest, and
    internal accounting is biased in this direction.
    """

    def __init__(self, parent, prop_assignments=None):
        """ Create a new state.  Optionally set the state assignment immediately,
            using `prop_assignments`. """

        if not isinstance(parent, StateCollection):
            raise TypeError("The parent of a State must be a StateCollection.")

        self.context = parent
        self.assignment = {}

        # Some optional meta-data
        self.state_id = None  # If you want to give the state a unique identifier
        self.goal_id = None   # Index of currently-pursued goal

        if prop_assignments is not None:
            self.setPropValues(prop_assignments)

    def getInputs(self, expand_domains=False):
        """ Return a dictionary of assignments to input propositions for this state.

            If `expand_domains` is True, return only the binary subpropositions
            for domains instead of the usual multivalent proposition. """

        return self.getPropValues(self.context.input_props, expand_domains)

    def getOutputs(self, expand_domains=False):
        """ Return a dictionary of assignments to output propositions for this state.

            If `expand_domains` is True, return only the binary subpropositions
            for domains instead of the usual multivalent proposition. """

        return self.getPropValues(self.context.output_props, expand_domains)

    def getAll(self, expand_domains=False):
        """ Return a dictionary of assignments to all propositions for this state.

            If `expand_domains` is True, return only the binary subpropositions
            for domains instead of the usual multivalent proposition. """

        assignments = self.getInputs(expand_domains)
        assignments.update(self.getOutputs(expand_domains))

        return assignments

    def satisfies(self, prop_assignments):
        """ Returns `True` iff the proposition settings in this state agree with all
            `prop_assignments`.  Any unspecified propositions are treated as don't-cares. """

        return all((self.getPropValue(k) == v for k, v in prop_assignments.iteritems()))

    def getPropValues(self, names, expand_domains=False):
        """ Return a dictionary of assignments to the propositions in `names`
            for this state.

            If `expand_domains` is True, return only the binary subpropositions
            for domains instead of the usual multivalent proposition. """

        prop_values = {p: self.getPropValue(p) for p in names}

        # If expand_domains is True, replace all domain propositions in the
        # return dictionary with their subpropositions
        if expand_domains:
            for n in names:
                domain = self.context.getDomainByName(n)
                if domain is None:
                    continue
                prop_values.update(domain.valueToPropAssignments(prop_values[n]))
                del prop_values[n]

        return prop_values

    def getPropValue(self, name):
        """ Return the value of the proposition `name` in this state.

            (Note: `expand_domains` is not supported here because it would
             entail returning multiple values.  Use getPropValues() for that.) """

        # OK, there are three possibilities for how we will evaluate the name:

        # 1) If this is a normal proposition name, and we know about it,
        #    just return its value directly
        if name in self.assignment:
            return self.assignment[name]

        # 2) If this is the name of a domain for which we only have the
        #    subpropositions, try to upconvert the subpropositions to a single
        #    multivalent proposition
        domain = self.context.getDomainByName(name)
        if domain is not None:
            # Try to calculate the value of the domain
            value = domain.propAssignmentsToValue(self.assignment)

            # Remove the subprops
            for subprop in domain.getPropositions():
                del self.assignment[subprop]

            # Add the multivalent prop
            self.assignment[name] = value

            return value

        # 3) Check to see if this is the name of a subproposition of any domain,
        #    in which case we can calculate the subproposition value
        parent_domain = self.context.getDomainOfProposition(name)
        if parent_domain is not None:
            return parent_domain.valueToPropAssignments(self.getPropValue(parent_domain.name))[name]

        # Otherwise, we'll have to throw an error
        raise ValueError("Proposition of name '{}' is undefined in this state".format(name))

    def setPropValue(self, prop_name, prop_value):
        """ Sets the assignment of propositions `prop_name` to `prop_value` in this state.
            A lot of sanity checking is performed to ensure the name and value are both appropriate. """

        # Check that this is a known prop_name
        if (prop_name not in self.context.input_props) and \
           (prop_name not in self.context.output_props) and \
           (self.context.getDomainOfProposition(prop_name) is None):
            raise ValueError("Unknown proposition/domain {!r}".format(prop_name))

        # Make sure that the value makes sense
        domain = self.context.getDomainByName(prop_name)
        if domain is None:
            if not isinstance(prop_value, bool):
                raise ValueError("Invalid value of {!r} for proposition {!r}: can only assign boolean values to non-Domain propositions".format(prop_value, prop_name))
        else:
            if prop_value not in domain.value_mapping:
                raise ValueError("Invalid value of {!r} for domain {!r}.  Acceptable values: {!r}".format(prop_value, prop_name, domain.value_mapping))

        # Store the value
        self.assignment[prop_name] = prop_value

    def setPropValues(self, prop_assignments):
        """ Update the assignments in this state according to `prop_assignments`.

            Any existing assignments to propositions not mentioned in `prop_assignments`
            are untouched. """

        for prop_name, prop_value in prop_assignments.iteritems():
            self.setPropValue(prop_name, prop_value)

    def stateToLTL(self, state, use_next=False, include_env=True, swap_io=False):
        """ swap_io is for the counterstrategy aut in mopsy """

        # TODO: fix this function

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
        return (self.context is other.context) and (self.assignment == other.assignment)

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
        self.input_props.append(domain.name)

    def addOutputDomain(self, domain):
        self.domains.append(domain)
        self.output_props.append(domain.name)

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

    def getDomainByName(self, name):
        return next((d for d in self.domains if d.name == name), None)

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
    states.addNewState({"region": "bedroom", "nearby_animal": "midshipman", "low_battery": True, "hypothesize": True, "experiment":False, "give_up":False})
    print "Inputs:", states.input_props
    print "Outputs:", states.output_props
    print "Domains:", states.domains
    for state in states:
        print state.getAll(expand_domains=True)
        print state

