import java.util.Vector;

import net.sf.javabdd.BDD;

import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.ModuleBDDField;
import edu.wis.jtlv.env.module.ModuleException;
import edu.wis.jtlv.env.module.ModuleWithWeakFairness;
import edu.wis.jtlv.env.spec.Operator;
import edu.wis.jtlv.env.spec.Spec;
import edu.wis.jtlv.env.spec.SpecBDD;
import edu.wis.jtlv.env.spec.SpecExp;

public class GROneParser {
	private static String exception_parse_stmt = "Each conjunct must be either: \n"
			+ "\t* Initial condition without any temporal operators or next statements.\n"
			+ "\t* Transition relation starting with operator \"[]\".\n"
			+ "\t* Winning condition starting with operators \"[]<>\", and without next statements.\n";

	public static void addReactiveBehavior(ModuleWithWeakFairness m, Spec[] spcs)
			throws Exception {
		for (Spec sp : spcs) {
			String exp_stmt = "Cannot parse specification \"" + sp + "\".\n"
					+ exception_parse_stmt;

			// ** 1. if it is just a simple BDD
			if (sp instanceof SpecBDD) {
				try {
					m.addInitial(((SpecBDD) sp).getVal());
				} catch (ModuleException e) {
					throw new Exception("Cannot add specification \"" + sp
							+ "\" as an initial condition");
				}
			} else {
				// ** 2. if it is a global specification.
				if (!(sp instanceof SpecExp))
					throw new Exception(exp_stmt);
				SpecExp spe = (SpecExp) sp;
				Spec child = spe.getChildren()[0];
				if (spe.getOperator() != Operator.GLOBALLY)
					throw new Exception(exp_stmt);
				// otherwise it is GLOBALLY
				if (child instanceof SpecBDD) {
					m.conjunctTrans(((SpecBDD) child).getVal());
				} else {
					// ** 3. if it is a global finally specification.
					if (!(child instanceof SpecExp))
						throw new Exception(exp_stmt);
					// otherwise it is SpecExp
					SpecExp childe = (SpecExp) child;
					if (childe.getOperator() != Operator.FINALLY)
						throw new Exception(exp_stmt);
					// otherwise it is FINALLY
					Spec grandchild = childe.getChildren()[0];
					if (!(grandchild instanceof SpecBDD))
						throw new Exception(exp_stmt);
					// otherwise it is SpecBDD
					m.addJustice(((SpecBDD) grandchild).getVal());
				}
			}
		}
	}

	public static void addPureReactiveBehavior(ModuleWithWeakFairness m,
			Spec[] spcs) throws Exception {

		BDD initial = Env.TRUE();
		BDD trans = Env.TRUE();
		for (Spec sp : spcs) {
			String exp_stmt = "Cannot parse specification \"" + sp + "\".\n"
					+ exception_parse_stmt;

			// ** 1. if it is just a simple BDD
			if (sp instanceof SpecBDD) {
				initial = initial.and(((SpecBDD) sp).getVal());
			} else {
				// ** 2. if it is a global specification.
				if (!(sp instanceof SpecExp))
					throw new Exception(exp_stmt);
				SpecExp spe = (SpecExp) sp;
				Spec child = spe.getChildren()[0];
				if (spe.getOperator() != Operator.GLOBALLY)
					throw new Exception(exp_stmt);
				// otherwise it is GLOBALLY
				if (child instanceof SpecBDD) {
					trans = trans.and(((SpecBDD) child).getVal());
				} else {
					// ** 3. if it is a global finally specification.
					if (!(child instanceof SpecExp))
						throw new Exception(exp_stmt);
					// otherwise it is SpecExp
					SpecExp childe = (SpecExp) child;
					if (childe.getOperator() != Operator.FINALLY)
						throw new Exception(exp_stmt);
					// otherwise it is FINALLY
					Spec grandchild = childe.getChildren()[0];
					if (!(grandchild instanceof SpecBDD))
						throw new Exception(exp_stmt);
					// otherwise it is SpecBDD
					m.addJustice(((SpecBDD) grandchild).getVal());
				}
			}
		}

		// /////////////////////////////////
		// roveri bug fix transformation.
		ModuleBDDField wasIField = null, conSField = null;
		// if (just_initial) {
		// wasIField = m.addVar(m.getName() + "_wasI");
		// } else if (just_safety) {
		// conSField = m.addVar(m.getName() + "_conS");
		// } else {
		wasIField = m.addVar(m.getName() + "_wasI");
		conSField = m.addVar(m.getName() + "_conS");
		// }

		// if ((!just_initial & !just_safety) & (wasIField == null))
		// throw new Exception("variable wasI must be declared.");
		// if ((!just_initial & !just_safety) & (conSField == null))
		// throw new Exception("variable conS must be declared.");

		// /////////////////////
		// // playing
		BDD wasITrue = Env.TRUE();
		BDD conSTrue = Env.TRUE();
		BDD nextConSTrue = Env.TRUE();
		BDD wasIMon = Env.TRUE();

		// if (!just_safety) {
		wasITrue = wasIField.getDomain().ithVar(1);
		wasIMon = wasIField.getDomain().buildEquals(wasIField.getOtherDomain());
		// }
		// if (!just_initial) {
		conSTrue = conSField.getDomain().ithVar(1);
		nextConSTrue = conSField.getOtherDomain().ithVar(1);
		// }

		m.addInitial(conSTrue.and(wasITrue.biimp(initial)));
		m.conjunctTrans(wasIMon.and(nextConSTrue.biimp(conSTrue.and(trans))));
		m.addJustice(m.popLastJustice().and(wasITrue).and(conSTrue));

	}

	static boolean just_initial = false;
	static boolean just_safety = false;

	public static Spec[] parseConjuncts(Spec sp) {
		Vector<Spec> res = new Vector<Spec>();
		parseConjunctsHelper(sp, res);
		Spec[] res_arr = new Spec[res.size()];
		res.toArray(res_arr);
		return res_arr;

	}

	private static void parseConjunctsHelper(Spec sp, Vector<Spec> res) {
		if (sp instanceof SpecExp) {
			if (((SpecExp) sp).getOperator() == Operator.AND) {
				Spec[] children = ((SpecExp) sp).getChildren();
				parseConjunctsHelper(children[0], res);
				parseConjunctsHelper(children[1], res);
				return;
			}
		}
		// otherwise just add this spec
		res.add(sp);
	}
}
