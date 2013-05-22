#!/usr/bin/python

import sys
import LTLParser
import logging
import re

# Allocate global parser
p = LTLParser.Parser()

class LTLFormulaType:
    """ For marking types of LTL subformulas.  `OTHER` generally means mixed. """
    INITIAL, SAFETY, LIVENESS, OTHER = range(4)

class LTLFormula(object):
    """ Wrapper around LTL formula parser, to allow for bidirectional
        conversions between string and parse-tree representations """

    def __init__(self, tree=None, *args, **kwds):
        super(LTLFormula, self).__init__(*args, **kwds)
        self.tree = tree

    @classmethod
    def fromString(cls, text):
        """ Create from a string """
        return cls(parseLTL(text))

    @classmethod
    def fromLTLFile(cls, filename):
        """ Takes in a file name and returns a tuple of two
            LTLFormulas: (assumptions, guarantees) """

        with open(filename) as f:
            file_data = f.read()

        m = re.search(r"LTLSPEC -- Assumptions\s*\((?P<assumptions>.*)\);\s*" + 
                      r"LTLSPEC -- Guarantees\s*\((?P<guarantees>.*)\);", \
                      file_data, re.DOTALL)

        if m is None:
            raise ValueError("Could not parse LTL file '%s'" % filename) 

        ltl_assumptions = m.group("assumptions")
        ltl_guarantees = m.group("guarantees")

        return cls.fromString(ltl_assumptions), cls.fromString(ltl_guarantees)

    def printTree(self, tree=None, terminals=None, indent=0):
        """Print a parse tree to stdout."""
        if terminals is None:
            terminals = p.terminals

        # Initialization for top-level of recursion
        if tree is None:
            tree = self.tree

        prefix = "    "*indent
        if tree[0] in terminals:
            print prefix + repr(tree)
        else:
            print prefix + unicode(tree[0])
            for x in tree[1:]:
                self.printTree(x, terminals, indent+1)

    #######################################################################
    # Note: these functions all assume input conforming strictly to GR(1) #
    #######################################################################

    def getType(self):
        """ Return type of formula represented """

        temporal_operators = self.getOutermostTemporalOperators(self.tree)

        if temporal_operators == '':
            return LTLFormulaType.INITIAL
        elif temporal_operators == 'G':
            return LTLFormulaType.SAFETY
        elif temporal_operators == 'GF':
            return LTLFormulaType.LIVENESS
        else:
            logging.warning("Unknown tree type: %r", temporal_operators)
            return LTLFormulaType.OTHER

    def getOutermostTemporalOperators(self, tree=None):
        """ Return the highest-scoped temporal operators around this formula"""

        # Initialization for top-level of recursion
        if tree is None:
            tree = self.tree

        if tree[0] != 'UnaryFormula':
            return ""

        if tree[1][0] == 'GloballyOperator':
            return 'G' + self.getOutermostTemporalOperators(tree[2])
        elif tree[1][0] == 'FinallyOperator':
            return 'F' + self.getOutermostTemporalOperators(tree[2])
        else:
            return ""

    def getConjunctsByType(self, kind):
        return [t for t in self.getConjuncts() if t.getType() == kind]

    def getConjuncts(self):
        if not self.tree[0] == "Conjunction":
            # This can happen if there is only one conjunct in the spec, for example
            logging.warning("Highest level not conjunction")
            return [self]
        
        return [LTLFormula(t) for t in self.tree[1:]]

    def __repr__(self):
        return '"' + self.__str__() + '"'

    def __str__(self):
        try:
            # If we are a conjunction at highest level
            # Note: This is done separately just so we can insert
            # linebreaks between each conjunct
            return " &\n".join(treeToString(t.tree, top_level=False) for t in self.getConjuncts())
        except ValueError:
            # Otherwise, if we are a single conjunct
            return treeToString(self.tree)
        

def tokenize(text):
    """ Lexer for the formulas """
    tokens = re.findall("(" + "|".join([re.escape(t) for t in p.terminals]) + \
                        "|[\w.]+)", text)

    return [(t,) if t in p.terminals else ('id', t) for t in tokens]

# =====================================================
# Simplify the specifications
# =====================================================
def clean_tree(tree):
    """ Cleans a parse tree, i.e. removes brackets and so on """

    if tree[0] in p.terminals:
        return tree

    if tree[0] == "Brackets":
        return clean_tree(tree[2])
    elif len(tree) == 2 and tree[0] in ["Implication", "Atomic", "Conjunction", "Biimplication", "Disjunction", "Xor", "BinaryTemporalFormula", "UnaryFormula"]:
        return clean_tree(tree[1])
    elif tree[0] == "AtomicFormula":
        if len(tree) != 2:
            raise ValueError("AtomicFormula must have length 2")
        return clean_tree(tree[1])
    elif tree[0] in ["Implication", "Conjunction", "Biimplication", "Disjunction", "Xor"]:
        return [tree[0], clean_tree(tree[1]), clean_tree(tree[3])]
    elif tree[0] == "BinaryTemporalFormula":
        return [tree[0], clean_tree(tree[1]), clean_tree(tree[2]), clean_tree(tree[3])]
    elif tree[0] == "UnaryFormula":
        return [tree[0], clean_tree(tree[1]), clean_tree(tree[2])]
    elif tree[0] in ["BinaryTemporalOperator", "UnaryTemporalOperator"]:
        # Remove the "superfluous indirection"
        return clean_tree(tree[1])
    elif tree[0] == "Assignment":
        # Flatten "id" case
        return [tree[0], [tree[1][1]]]
    else:
        return [tree[0]] + [clean_tree(x) for x in tree[1:]]

def flatten_as_much_as_possible(tree):
    """ Flattens nested disjunctions/conjunctions """
    # Ground case?
    if len(tree) == 1 or isinstance(tree, basestring):
        return tree

    tree = [flatten_as_much_as_possible(t) for t in tree]

    FLATTENABLE_TYPES = ["Conjunction", "Disjunction", "Xor"]

    if tree[0] in FLATTENABLE_TYPES:
        parts = [tree[0]]
        for a in tree[1:]:
            if a[0] == tree[0]:
                parts.extend(a[1:])
            else:
                parts.append(a)

        return parts

    # Every other case
    return tree


# =====================================================
# The parsing function
# =====================================================
def parseLTL(ltlTxt):
    try:
        tokens = tokenize(ltlTxt)
        tree = p.parse(tokens)
    except p.ParseErrors as exc:
        for t, e in exc.errors:
            if t[0] == p.EOF:
                logging.error("Formula end not expected here")
                continue

            if len(e) == 1:
                logging.error("Error in LTL formula: %r", ltlTxt)
                logging.error("Expected %r, but found %r", e[0], t[0])
            else:
                logging.error("Error in LTL formula: %r", ltlTxt)
                logging.error("Could not parse %r: ", t[0])
                logging.error("Wanted a token of one of the following forms: %r", e)
        raise

    # Post-process
    cleaned_tree = clean_tree(tree)
    simplified_tree = flatten_as_much_as_possible(cleaned_tree)

    return simplified_tree

def treeToString(tree, top_level=True):
    """
    Flatten an LTL tree back to a string
    """

    # Mapping of n-ary operator types to their string representations
    n_ary_operators = {"Conjunction": " & ",
                       "Disjunction": " | ",
                       "Implication": " -> ",
                       "Biimplication": " <-> "}

    # We need to force parentheses for some operators, even if they are unary
    requires_parens = ["NextOperator", "GloballyOperator", "FinallyOperator"]

    if len(tree) == 1:
        # Terminals and assignments, etc.
        return tree[0]
    elif tree[0] in n_ary_operators.keys():
        # Join the subparts of the n-ary operator
        chunk = n_ary_operators[tree[0]].join(treeToString(t, top_level=False) for t in tree[1:])
        
        # Group with parenthese unless we are at the top level, in which case they are unnecessary
        if not top_level:
            chunk = "(" + chunk + ")"

        return chunk
    elif tree[0] == "UnaryFormula" and tree[1][0] in requires_parens:
        child = treeToString(tree[2], top_level=False)
        # Add parentheses only if necessary
        if not (child.startswith("(") and child.endswith(")")) \
           and not (tree[1][0] == "GloballyOperator" and tree[2][0] == "UnaryFormula" and tree[2][1][0] == "FinallyOperator"):
           # ^^^ HACK: To be backwards compatible, we want to avoid [](<>(something))
            child = "(" + child + ")"
        return treeToString(tree[1], top_level=False) + child
    else:
        return "".join(treeToString(t, top_level=False) for t in tree[1:])
        
if __name__ == "__main__":
    ### Test code:

    fname = "../../examples/firefighting/firefighting.ltl"

    assumptions, guarantees = LTLFormula.fromLTLFile(fname)

    print assumptions.getConjunctsByType(LTLFormulaType.INITIAL)
    print assumptions.getConjunctsByType(LTLFormulaType.SAFETY)
    print assumptions.getConjunctsByType(LTLFormulaType.LIVENESS)

    print assumptions 

    print guarantees.getConjunctsByType(LTLFormulaType.INITIAL)
    print guarantees.getConjunctsByType(LTLFormulaType.SAFETY)
    print guarantees.getConjunctsByType(LTLFormulaType.LIVENESS)

    print guarantees

    #x = assumptions.getConjuncts()
    #x.append(LTLFormula.fromString("[]<>whatever"))
    #createLTLfile("test.ltl", x, g)
