import sys
import re
import time
import pycudd

# README:
# This module requires a modified version of pycudd
# TODO: commit patch

#class BDDStrategy(Strategy):
class BDDStrategy(object):
    """ TODO WRITE SOMETHING HERE
    """

    def __init__(self, *args, **kwds):
        super(BDDStrategy, self).__init__(*args, **kwds)
        self.strategy = None
        self.varNameToBDD = {}
        self.BDDToVarName = {}
        self.varNumToVarName = []
        self.jxVars = []
        self.stratTypeVar = None
    
        self.mgr = pycudd.DdManager()
        self.mgr.SetDefault()

        # Choose a timer func with maximum accuracy for given platform
        if sys.platform in ['win32', 'cygwin']:
            self.timer_func = time.clock
        else:
            self.timer_func = time.time

    @classmethod
    def fromFile(cls, filename):
        obj = cls()
        obj.loadBDDFile(filename)
        return obj

    def loadBDDFile(self, filename):
        print "Loading BDD strategy from file '{}'...".format(filename)
        a = pycudd.DdArray(1)
        tic = self.timer_func()

        self.mgr.AddArrayLoad(pycudd.DDDMP_ROOT_MATCHLIST,
                              None,
                              pycudd.DDDMP_VAR_MATCHIDS,
                              None,
                              None,
                              None, 
                              pycudd.DDDMP_MODE_TEXT,
                              filename, None, a)

        self.strategy = self.mgr.addBddPattern(a[0])

        with open(filename, 'r') as f: 
            loading_vars = False
            for line in f:
                if line.startswith("# Variable names:"):
                    # signals start of variable names section
                    loading_vars = True
                elif loading_vars:
                    # parse each variable name
                    m = re.match(r"^#\s*(?P<num>\d+)\s*:\s*<(?P<name>\w+'?)>", line)
                    if m is not None:
                        varname = m.group("name")
                        if varname == "jx":
                            self.jxVars.append(~self.mgr.IthVar(int(m.group("num"))))
                        elif varname == "strat_type":
                            self.stratTypeVar = ~self.mgr.IthVar(int(m.group("num")))
                        else:
                            # split into past and future, maybe for future use reverse mapping
                            if varname.endswith("'"):
                                self.BDDToVarName[~self.mgr.IthVar(int(m.group("num")))] = varname
                            else:
                                self.varNameToBDD[varname] = ~self.mgr.IthVar(int(m.group("num")))

                        self.varNumToVarName.append(varname) # TODO: check for consecutivity
                    else:
                        loading_vars = False
                elif line.startswith("# root needs negating"):
                    # check for root negation
                    print "negating root"
                    self.strategy = ~self.strategy

        toc = self.timer_func()
        print "Loaded in {} seconds.".format(toc-tic)

    def printStrategy(self):
        self.strategy.PrintMinterm()

    def stateToBDD(self, state):
        # TODO: actually use propvalues in state
        bdd = self.mgr.ReadOne()
        # for testing, use all == 0
        for k,v in self.varNameToBDD.iteritems():
            bdd = bdd & ~v

        return bdd 

    def getTransitions(self, state, jx, strat_type):
        # 0. The strategies that do not change the justice pursued
        # 1. The strategies that change the justice pursued
        if strat_type == "Y":
            strat_type_bdd = ~self.stratTypeVar
        elif strat_type == "Z":
            strat_type_bdd = self.stratTypeVar
        else:
            print "bad strat type", strat_type
            return None
        cand = self.strategy & state & self.getJxBDD(jx) & strat_type_bdd
        return cand

    def getJxBDD(self, jx):
        jx_bdd = self.mgr.ReadOne()
        # TODO: safety checks
        for i, bit in enumerate(("{:0"+str(len(self.jxVars))+"b}").format(jx)[::-1]): # lesser significant bits have lower varids
            if bit == "0":
                jx_bdd &= ~self.jxVars[i]
            elif bit == "1":
                jx_bdd &= self.jxVars[i]
            else:
                print "this bit is wack", bit
        return jx_bdd

    def cubeToString(self, cube):
        cube = list(cube)
        for i, v in enumerate(cube): 
            if v == 0:
                cube[i] = "{}".format(self.varNumToVarName[i])
            elif v == 1:
                cube[i] = "!{}".format(self.varNumToVarName[i])
            elif v == 2:
                #cube[i] = "({})".format(self.varNumToVarName[i])
                cube[i] = "--"
            else:
                print "this cube is wack", cube
        return ", ".join(cube)

    def printSats(self, bdd):
        pycudd.set_iter_meth(0)
        for cube in bdd:
            print self.cubeToString(cube)
            #print pycudd.cube_tuple_to_str(cube)

    # getTransitions or whaterver else FSA does
        



if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Usage: {} [BDD filename]".format(sys.argv[0])
        sys.exit()

    strategy = BDDStrategy.fromFile(sys.argv[1])
    #strategy.printStrategy() 
    print " ".join(strategy.varNumToVarName)
    curr_state = strategy.stateToBDD(None)
    strategy.printSats(curr_state)
    strategy.printSats(strategy.getJxBDD(1))
    tr = strategy.getTransitions(curr_state, 1, "Y")
    strategy.printSats(tr)
    #print "---"
    #strategy.printSats(strategy.strategy)
