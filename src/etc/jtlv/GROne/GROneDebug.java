import net.sf.javabdd.BDD;
import net.sf.javabdd.BDDVarSet;
import net.sf.javabdd.BDD.BDDIterator;
import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.SMVModule;
import edu.wis.jtlv.env.spec.Spec;
import java.io.File;
import java.io.PrintStream;
import edu.wis.jtlv.lib.FixPoint;
import edu.wis.jtlv.old_lib.games.GameException;



public class GROneDebug {

	/**
	 * @param args
	 * @throws Exception
	 */
	public static void main(String[] args) throws Exception {
		// uncomment to use a C BDD package
		//System.setProperty("bdd", "buddy");

		GROneParser.just_initial = true;
		// GRParser.just_safety = true;

        // Check that we have enough arguments
        if (args.length < 2) {
            System.err.println("Usage: java GROneMain [smv_file] [ltl_file]");
            System.exit(1);
        }

        Env.loadModule(args[0]);
        Spec[] spcs = Env.loadSpecFile(args[1]);

        // Figure out the name of our output file by stripping the spec filename extension and adding .aut
        String out_filename = args[1].replaceAll("\\.[^\\.]+$",".aut");

		// constructing the environment module.
		SMVModule env = (SMVModule) Env.getModule("main.e");
		Spec[] env_conjuncts = GROneParser.parseConjuncts(spcs[0]);
		GROneParser.addReactiveBehavior(env, env_conjuncts);
		// GRParser.addPureReactiveBehavior(env, env_conjuncts);

		// constructing the system module.
		SMVModule sys = (SMVModule) Env.getModule("main.s");
		Spec[] sys_conjuncts = GROneParser.parseConjuncts(spcs[1]);
		GROneParser.addReactiveBehavior(sys, sys_conjuncts);
		//GROneParser.addPureReactiveBehavior(sys, sys_conjuncts);
		
		analyze(env, sys);
		justiceChecks(env, sys);
	}
	
	public static void analyze(SMVModule env, SMVModule sys) {
		//The output here should really go into a file, to be read from specEditor.py -- Vasu
		
		BDD sys_init = sys.initial();
		BDD env_init = env.initial();
		BDD all_init = sys_init.and(env_init);	 
		 
		BDD init_cox,init_cox_Rev, init_sl;

		FixPoint<BDD> iterSl;
		
		
		 init_sl = Env.FALSE();
		 for (iterSl = new FixPoint<BDD>(); iterSl.advance(init_sl);) {			 
			 init_sl = init_sl.or(env.yieldStates(sys, init_sl));
		 }
		 init_cox = init_sl.id().and(all_init);

		 init_sl = Env.FALSE();
		 for (iterSl = new FixPoint<BDD>(); iterSl.advance(init_sl);) {
			 init_sl = (init_sl).or((env.yieldStates(sys, init_sl.not())).not());
		 }
		 
		 
		 init_cox_Rev = init_sl.id().and(all_init);
		 
		 System.out.println("cox Y REV = "+ init_cox_Rev); //TRUE if unsat
		 
		 System.out.println("cox Y = "+ init_cox); //TRUE if unsat
		 
		 if ((sys.initial().and(sys.trans())).isZero()) {
			 System.out.println("Unsat transitions.");
		 }	
		 	
		 else if (sys.initial().isZero()) {
			 System.out.println("Unsat initial conditions.");
		 
		 } else {
		 	 
			 for (int i = 0; i < sys.justiceNum(); i++) {
				 if (sys.justiceAt(i).isZero()) 
					 System.out.println("Unsat goal number " + i);
				 else if ((sys.trans().and(Env.prime(sys.justiceAt(i))).isZero()) | sys.trans().and(sys.justiceAt(i)).isZero())
					 System.out.println("Unsat between trans and goal number " + i);
				
			 }
			 			 
		 }
		  
		  if (env.trans().isZero()) {
				 System.out.println("REV Unsat transitions.");
			 }		 
		  else if (env.initial().isZero()) {
				 System.out.println("REV Unsat initial conditions.");
			  
		  } else {
				 	 
				 for (int i = 0; i < env.justiceNum(); i++) {
					 if (env.justiceAt(i).isZero()) 
						 System.out.println("REV Unsat goal number " + i);
					 else if ((env.trans().and(Env.prime(env.justiceAt(i))).isZero()) | (env.trans().and(env.justiceAt(i))).isZero()) 
						 System.out.println("REV Unsat between trans and goal number " + i);
					
				 }
				 			 
		  }
	}
	
	
	public static void justiceChecks(SMVModule env, SMVModule sys) throws GameException  {
		//The output here should really go into a file, to be read from specEditor.py -- Vasu
		
		BDD all_init = sys.initial().and( env.initial());
		BDD counter_exmple;
		GROneGame g;
		
		
		for (int i = 1; i <= sys.justiceNum(); i++){
			 g = new GROneGame(env,sys, i, env.justiceNum());
			 counter_exmple = g.envWinningStates().and(all_init);
			 if (!counter_exmple.isZero()&& (!sys.justiceAt(i-1).equals(Env.TRUE()))) {
				 System.out.println("sl Y = "+ (i-1));
				 System.out.println("System is unrealizable because of justice "+ (i-1));
				 i = sys.justiceNum() + 1;
			 }
		}
		
		BDD prev;
		g = new GROneGame(env,sys, sys.justiceNum(), env.justiceNum());
		counter_exmple = g.envWinningStates().and(all_init);		 
		for (int i = env.justiceNum(); i >=1; i--){
			 prev = counter_exmple;
			 g = new GROneGame(env,sys, sys.justiceNum(), i);
			 counter_exmple = g.envWinningStates().and(all_init);
			 System.out.println(prev);
			 
			 if (counter_exmple.isZero() && (!env.justiceAt(i-1).equals(Env.TRUE()))) {// && (!prev.isZero())) {
				 System.out.println("sl Y REV = "+ (i-1));
				 System.out.println("Environment is unrealizable because of justice "+ (i-1));
				 i = 0;
			 } 
		}		
	}
}
