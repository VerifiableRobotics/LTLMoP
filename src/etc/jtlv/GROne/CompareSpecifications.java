/* This program is meant to provide a way to do regression testing on LTL output,
 * by allowing the comparison of specifications as BDDs, making them invariant to
 * minor changes in "wording" of the exact LTL.
 *
 * Returns 1 if the specifications are different, 0 if they are equivalent.
 */

import net.sf.javabdd.BDD;
import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.SMVModule;
import edu.wis.jtlv.env.spec.Spec;

public class CompareSpecifications {

    /* We need to have a way to store the specification BDDs outside of the JTLV module,
       because otherwise loading the second specification will smash the first one */
    private static class SinglePlayerSpecificationAsBDDs {
        public BDD init, trans;
        public BDD[] goals;
        public int num_goals;
    }

    private static boolean specsAreDifferent = false;

	public static void main(String[] args) throws Exception {
        // Check that we have enough arguments
        if (args.length < 3) {
            System.err.println("Usage: java CompareSpecifications <smv_file> <ltl_file_1> <ltl_file_2>");
            System.exit(1);
        }                

        // Load SMV and LTL files
        Env.loadModule(args[0]);
        SinglePlayerSpecificationAsBDDs[] spec1, spec2;
        spec1 = loadSpecification(args[1]);
        spec2 = loadSpecification(args[2]);

        System.out.println("=== Comparing environment assumptions...");
        compareHalfSpecification(spec1[0], spec2[0]);

        System.out.println("\n=== Comparing system requirements...");
        compareHalfSpecification(spec1[1], spec2[1]);

        if (specsAreDifferent) {
            System.out.println("SPECIFICATIONS ARE NOT EQUIVALENT!");
        } else {
            System.out.println("The two specifications appear to be equivalent.");
        }

        // Give a useful return value for scripts
        System.exit(specsAreDifferent ? 1 : 0);
	}

    private static void compareHalfSpecification(SinglePlayerSpecificationAsBDDs s1, SinglePlayerSpecificationAsBDDs s2) {
        System.out.print(">> Initial conditions:");
        compareBDDs(s1.init, s2.init);
        System.out.print(">> Transitions:");
        compareBDDs(s1.trans, s2.trans);
        System.out.println(">> Goals:");
        if (s1.num_goals == s2.num_goals) {
            for (int jx = 0; jx < s1.num_goals; jx++) {
                System.out.print("   -> jx = " + jx + ":");
                compareBDDs(s1.goals[jx], s2.goals[jx]);
            }
        } else {
            System.out.println("   DISCREPANCY DETECTED! Number of goals differs :(");
            specsAreDifferent = true;
        }
    }

    private static void compareBDDs(BDD a, BDD b) {
        BDD diff = a.and(b.not());
        if (a.equals(b)) {
            System.out.println(" Identical!");
        } else {
            System.out.println(" DISCREPANCY DETECTED!  Spec1 & !Spec2 is:");
            diff.printSet();
            specsAreDifferent = true;
        }
    }

    /* Remove any traces of a previously-loaded specification from the module before using it */
    private static SMVModule getCleanModule(String name) throws Exception {
		SMVModule m = (SMVModule) Env.getModule(name);
        
        // For some unknown reason, removeAllIniRestrictions() and removeAllTransRestrictions() don't seem to work
        m.setInitial(Env.TRUE());
        m.disjunctTrans(Env.TRUE());

        // Gotta clear these out one by one, apparently
        while (m.justiceNum() > 0) {
            m.popLastJustice();
        }

        return m;
    }

    private static SinglePlayerSpecificationAsBDDs getBDDSpecification(String module_name, Spec spec_part) throws Exception {
		SMVModule m = getCleanModule(module_name);

		Spec[] conjuncts = GROneParser.parseConjuncts(spec_part);
		GROneParser.addReactiveBehavior(m, conjuncts);

        SinglePlayerSpecificationAsBDDs bdd_spec;
        bdd_spec = new SinglePlayerSpecificationAsBDDs();
        bdd_spec.init = m.initial().id();
        bdd_spec.trans = m.trans().id();
        bdd_spec.num_goals = m.justiceNum();
        bdd_spec.goals = new BDD[bdd_spec.num_goals];
        for (int jx = 0; jx < bdd_spec.num_goals; jx++) {
            bdd_spec.goals[jx] = m.justiceAt(jx).id();
        }
       
        return bdd_spec; 
    }

    private static SinglePlayerSpecificationAsBDDs[] loadSpecification(String filename) throws Exception {
        Spec[] spcs = Env.loadSpecFile(filename);
        SinglePlayerSpecificationAsBDDs[] bdd_specs = new SinglePlayerSpecificationAsBDDs[2];
        
        bdd_specs[0] = getBDDSpecification("main.e", spcs[0]);
        bdd_specs[1] = getBDDSpecification("main.s", spcs[1]);

        return bdd_specs;
    }

}
