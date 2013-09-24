import net.sf.javabdd.BDD;
import java.util.ArrayList;
import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.SMVModule;
import edu.wis.jtlv.env.spec.Spec;
import edu.wis.jtlv.env.spec.Operator;
import edu.wis.jtlv.env.spec.SpecBDD;
import edu.wis.jtlv.env.spec.SpecExp;

public class SpecSubsets {
    public enum ConjunctType {
        INITIAL, SAFETY, LIVENESS, OTHER;
    }

	public static void main(String[] args) throws Exception {
        // Check that we have enough arguments
        if (args.length < 2) {
            System.err.println("Usage: java SpecSubsets <smv_file> <ltl_file>");
            System.exit(1);
        }                

        // Load SMV and LTL files
        Env.loadModule(args[0]);
        Spec[] spcs = Env.loadSpecFile(args[1]);

        // Parse the specs
		Spec[] e_conjuncts = GROneParser.parseConjuncts(spcs[0]);
		Spec[] s_conjuncts = GROneParser.parseConjuncts(spcs[1]);
		
        // Load all of the environment spec
		SMVModule m_env = (SMVModule) Env.getModule("main.e");
		GROneParser.addReactiveBehavior(m_env, e_conjuncts);

		SMVModule m_sys = (SMVModule) Env.getModule("main.s");
        // Load all of the system initials and goals
        GROneParser.addReactiveBehavior(m_sys, getConjunctsByType(s_conjuncts, ConjunctType.INITIAL));
        GROneParser.addReactiveBehavior(m_sys, getConjunctsByType(s_conjuncts, ConjunctType.LIVENESS));
 
        m_sys.initial().printSet();
        
        // Incrementally load the system safeties
        for (Spec safety_conjunct : getConjunctsByType(s_conjuncts, ConjunctType.SAFETY)) {
            clearSpecPartFromModule(m_sys, ConjunctType.SAFETY); // Remove this line if you want to accumulate
            System.out.println("Adding " + safety_conjunct);
            GROneParser.addReactiveBehavior(m_sys, new Spec[]{safety_conjunct});
            m_sys.trans().printSet();
            System.out.println("=====================================");
        }
	}

    public static Spec[] getConjunctsByType(Spec[] sp, ConjunctType t) throws Exception {
        ArrayList<Spec> conjuncts = new ArrayList<Spec>();
        
        for (Spec conjunct : sp) {
            if (getConjunctType(conjunct) == t) {
                conjuncts.add(conjunct);
            }
        }
          
        return conjuncts.toArray(new Spec[0]);
    }

    public static String getOutermostTemporalOperators(Spec sp) throws Exception {
        // Stop recursion when we reach a pure BDD
        if (sp instanceof SpecBDD) {
            return "";
        }

        // This should never happen
        if (!(sp instanceof SpecExp)) {
            throw new Exception("Encountered subconjunct that is neither SpecBDD or SpecExp");
        }
    
        // Cast to SpecExp
        SpecExp spe = (SpecExp) sp;
        
        // Add the temporal operator we see and recurse
        if (spe.getOperator() == Operator.GLOBALLY) {
            return "G" + getOutermostTemporalOperators(spe.getChildren()[0]);
        } else if (spe.getOperator() == Operator.FINALLY) {
            return "F" + getOutermostTemporalOperators(spe.getChildren()[0]);
        } else {
            throw new Exception("Encountered SpecExp that is neither GLOBALLY or FINALLY");
        }
    }

    public static ConjunctType getConjunctType(Spec sp) throws Exception {
        String operators = getOutermostTemporalOperators(sp);

        if (operators.equals("")) {
            return ConjunctType.INITIAL;
        } else if (operators.equals("G")) {
            return ConjunctType.SAFETY;
        } else if (operators.equals("GF")) {
            return ConjunctType.LIVENESS;
        } else {
            return ConjunctType.OTHER;
        }
    }

    private static void clearSpecPartFromModule(SMVModule m, ConjunctType t) throws Exception {
        // For some unknown reason, removeAllIniRestrictions() and removeAllTransRestrictions() don't seem to work
        if (t == ConjunctType.INITIAL) {
            m.setInitial(Env.TRUE());
        } else if (t == ConjunctType.SAFETY) {
            m.disjunctTrans(Env.TRUE());
        } else if (t == ConjunctType.LIVENESS) {
            // Gotta clear these out one by one, apparently
            while (m.justiceNum() > 0) {
                m.popLastJustice();
            }
        }
    }
}
