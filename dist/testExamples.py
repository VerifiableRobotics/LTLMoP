"""
Checks that all examples load and synthesize successfully.
"""

import unittest
import glob
import sys, os
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..","src","lib"))
import specCompiler


class TestExample(unittest.TestCase):
    def __init__(self, spec_filename):
        super(TestExample, self).__init__()
        self.spec_filename = spec_filename

    def runTest(self):
        title_str = "#### Testing project '{0}' ####".format(self.spec_filename)

        print

        if sys.platform not in ['win32', 'cygwin']:
            print "\033[41m"  # red background color

        print "#"*len(title_str)
        print title_str
        print "#"*len(title_str),

        if sys.platform not in ['win32', 'cygwin']:
            print "\033[0m"   # end coloring

        print

        c = specCompiler.SpecCompiler(self.spec_filename)
        c_out = c.compile()

        self.assertIsNotNone(c_out, msg="Compilation failed due to parser error")
        realizable, realizableFS, output = c_out

        print output

        expectedToBeUnrealizable = ("unsynth" in self.spec_filename) or \
                                   ("unreal" in self.spec_filename) or \
                                   ("unsat" in self.spec_filename)

        if expectedToBeUnrealizable:
            self.assertFalse(realizable, msg="Specification was realizable but we did not expect this")
        else:
            self.assertTrue(realizable, msg="Specification was unrealizable")

        #self.assertEqual(function_to_test(self.input), self.output)

def getTester(spec_filename):
    class NewTester(TestExample): pass
    NewTester.__name__ = "TestExample_" + spec_filename.replace(".","_").replace("\\","_").replace("/","_")
    return NewTester(spec_filename)
    
def suite():
    suite = unittest.TestSuite()
    suite.addTests(getTester(fname) for fname in glob.iglob(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..","src","examples","*","*.spec")))
    return suite

if __name__ == '__main__':
    unittest.TextTestRunner().run(suite())
