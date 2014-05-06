#!/usr/bin/env python
""" This module provides helper functions for parsing user-defined
    method calls.  Python's built-in AST machinery is used because
    regular expressions are not powerful enough to deal with arbitrarily
    nested quotes, parentheses, etc.
"""

import ast
import re
import types
from collections import namedtuple
import logging
import globalConfig
from itertools import chain

class CallDescriptor(namedtuple("CallDescriptor",
                                "name, args, start_pos, end_pos")):
    """
    A lightweight class for storing information about method calls
    contained in a string.
      - `name` is a list of the subparts of the called method's name (i.e. the
        full name, split at periods)
      - `args` is a dict of the keyword args passed to the method
      - `start_pos` and `end_pos` are the indices in the input string where the
      call text starts and ends, respectively.
    """
    pass


def parseCallString(text, mode="single", make_call_function=None):
    """ Inputs: 
        - A string of calls, e.g. "a.b(c=1, d=2)", optionally joined by the boolean
          operators "and" or "or".
        - The `mode` option dictates how boolean operators are handled.  If `mode`
          is "sensor", AND is the expected logical conjunct; if "actuator", AND
          means "and then do"; if "single", the presence of any Boolean operator
          will throw a SyntaxError.
        - Also optionally accepts a `make_call_function` function that
          takes a CallDescriptor and returns a function that takes any kwargs
          and returns a boolean result.

        Outputs:
        - A list of CallDescriptors (in the order they appeared in the input string).
          See the CallDescriptor definition, and the doctests for examples.
        - A function that returns the boolean result of effectively evaluating
          the original input string with `make_call_function` applied to each call.
          Any kwargs passed to this function will be passed to each subfunction.
          If `make_call_function` is `None`, this function will be `None`.

        Example:
        >>> parseCallString("a.b(c=1, d=2)")
        ([CallDescriptor(name=['a', 'b'], args={'c': 1, 'd': 2}, start_pos=0, end_pos=12)], None)
    """

    # Parse into AST
    tree = ast.parse(text)

    # Check that we were given reasonable input
    if not (isinstance(tree, ast.Module)
            and len(tree.body) == 1
            and isinstance(tree.body[0], ast.Expr)):
        raise SyntaxError("Exactly one expression is required")

    # Start the recursion from the first & only Expr (which itself is always
    # wrapped in a top-level Module)
    try:
        call_list, f = parseCallTree(tree.body[0].value, mode, make_call_function)
    except SyntaxError:
        logging.error("Error while parsing line {!r}".format(text))
        raise

    if f is not None and isinstance(f, types.LambdaType):
        # Give the highest-level function some useful metadata
        # TODO: hsub can give this a better name
        f.func_name = re.sub("\W", "_", text)
        f.__doc__ = text

    # Collapse the itertools chain
    call_list = list(call_list)

    # Do some sneaky calculations to figure out end_pos values
    # because AST will only give us start_pos
    for k in xrange(len(call_list)):
        # Start out assuming this call ends right at the beginning of the next
        # call (or the end of the string, for the last call)
        if k+1 < len(call_list):
            end_pos = call_list[k+1].start_pos
        else:
            end_pos = len(text)-1

        # Move backwards until we find the closing parenthesis of this call
        # Yeah, talk about elegant! :\
        while text[end_pos] != ")":
            end_pos -= 1
        
        # Weird result of using namedtuples
        # Note that we add 1 to mimic the way regex groups give end()
        call_list[k] = call_list[k]._replace(end_pos=end_pos+1)

    return call_list, f

def parseCallTree(tree, mode, make_call_function):
    """ This function contains the recursive parts of parseCallString. """

    if isinstance(tree, ast.BoolOp):
        # Make sure we were expecting this
        if mode == "single":
            raise SyntaxError("Boolean operators are not permitted in 'single' parsing mode.")

        # Evaluate our children
        subresults = [parseCallTree(t, mode, make_call_function) for t in tree.values]

        # Combine all the CallDescriptors from our children
        joined_calls = chain.from_iterable(r[0] for r in subresults)

        # If make_call_function is None, we don't need to construct f so
        # we are done here
        if make_call_function is None:
            return joined_calls, None

        # Construct a function appropriately joining our subfunctions
        if isinstance(tree.op, ast.And):
            if mode == "sensor":
                f = lambda **kwargs: all(r[1](**kwargs) for r in subresults)
            elif mode == "actuator":
                # For actuators, we treat "and" as "and next..."
                # We can return a list of the return values, but it's probably not useful
                f = lambda **kwargs: [r[1](**kwargs) for r in subresults]
        elif isinstance(tree.op, ast.Or):
            if mode == "sensor":
                f = lambda **kwargs: any(r[1](**kwargs) for r in subresults)
            elif mode == "actuator":
                raise SyntaxError("OR operator is not permitted in actuators because it doesn't make sense.")

        return joined_calls, f

    elif isinstance(tree, ast.Call):
        # Calculate the full name of the function
        name_parts = []
        subtree = tree.func
        while isinstance(subtree, ast.Attribute):
            name_parts.insert(0, subtree.attr)
            subtree = subtree.value
        name_parts.insert(0, subtree.id)

        # Extract the function arguments using literal_eval
        kwargs = {}
        for kw in tree.keywords:
            try:
                kwargs[kw.arg] =  ast.literal_eval(kw.value)
            except ValueError:
                name = ".".join(name_parts)
                raise ValueError("Invalid value for argument {!r} of handler/method named {!r}".format(kw.arg, name))

        # Make a CallDescriptor
        cd = CallDescriptor(name=name_parts,
                            args=kwargs,
                            start_pos=tree.col_offset,
                            end_pos=None) # We can't get end_pos from AST
                                          # so we'll calculate this later

        if make_call_function is not None:
            # Let's get the function call for this CallDescriptor
            f = make_call_function(cd)
        else:
            f = None

        return [cd], f
    else:
        raise SyntaxError("Encountered unexpected node of type {}".format(type(tree)))

if __name__ == "__main__":
    logging.info("Running doctests...")
    import doctest
    doctest.testmod()

    logging.info("Running other tests...")
    def make_fake_function(cd):
        print "making {}".format(cd.name)
        def fake_function(**kwargs):
            args = kwargs
            args.update(cd.args)
            print "call: {}({})".format(cd.name, args)
            return True
        return fake_function

    test_strings = ["a.b(c=1,     d =2)   and  f(1) or  jim.likes.pizza(topping=['stuff'])"]
    for s in test_strings:
        cds, f = parseCallString(s, mode="sensor", make_call_function=make_fake_function)
        print cds
        print [s[cd.start_pos:cd.end_pos] for cd in cds]
        print f
        print f(initial=True)

    
