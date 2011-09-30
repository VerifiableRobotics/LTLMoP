import net.sf.javabdd.BDD;
import net.sf.javabdd.BDDVarSet;
import net.sf.javabdd.BDD.BDDIterator;
import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.SMVModule;
import edu.wis.jtlv.env.spec.Spec;
import java.io.File;
import java.io.PrintStream;
import java.io.OutputStream;
import java.io.FileWriter;
import java.io.BufferedWriter;
import edu.wis.jtlv.lib.FixPoint;
import edu.wis.jtlv.old_lib.games.GameException;
import edu.wis.jtlv.env.spec.SpecBDD;
import edu.wis.jtlv.lib.AlgRunnerThread;
import edu.wis.jtlv.lib.AlgResultI;
import edu.wis.jtlv.lib.mc.tl.LTLModelCheckAlg;



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
		
		analyze(env, sys, "default.txt");
		
	}
	
	public static void analyze(SMVModule env, SMVModule sys, String filename) {
		int explainSys=0, explainEnv = 0; //keep track of explanations ot avoid redundancy
		
		BDD sys_init = sys.initial();
		BDD env_init = env.initial();
		BDD all_init = sys_init.and(env_init);	 
		 
		BDD envUnreal, sysUnreal;
		BDD envUnsat, sysUnsat;

		FixPoint<BDD> iter;
		
		String debugInfo = "";
		
			
		 BDD cox = Env.FALSE();
		 for (iter = new FixPoint<BDD>(); iter.advance(cox);) {			 
			 cox = cox.or(env.yieldStates(sys, cox));
		 }
		 envUnreal = cox.id().and(all_init);

		 cox = Env.FALSE();
		 for (iter = new FixPoint<BDD>(); iter.advance(cox);) {
			 cox = cox.or((env.yieldStates(sys, cox.not())).not());			 
		 }	 
		 
		 sysUnreal = cox.id().and(all_init);
		 
		 
		 
		 if (sys.initial().isZero()) {
			 debugInfo += "SysInit UNSAT" + "\n";
			 explainSys = 1;
		 
		 } else if ((sys.initial().and(sys.trans())).isZero()) {
			 debugInfo += "SysTrans UNSAT" + "\n";
			 explainSys = 1;
		 } 
		  
		 if (env.initial().isZero()) {
			  debugInfo += "EnvInit UNSAT" + "\n";
			  explainEnv = 1;	 
		 } else if ((env.initial().and(env.trans())).isZero()) {
			  debugInfo += "EnvTrans UNSAT" + "\n";
			  explainEnv = 1;
		 } 
		 
		 GROneGame g = null;
		 
		 try { 
			  	g = new GROneGame(env,sys, sys.justiceNum(), env.justiceNum());		
		  }	catch (Exception e){//Catch exception if any
					System.err.println("Error: " + e.getMessage());
		  }
		 
		  BDD counter_example = g.envWinningStates().and(all_init);
		  System.out.println("counter_example = " + counter_example);
		  try { 
			  if (!counter_example.isZero()) {
				//checking for multi-step unsatisfiability between sys transitions and initial condition					 
				 if (g.calculate_counterstrategy(counter_example, false, false)) { 			 
					 debugInfo += "SysInitTrans UNSAT " + "\n";
					 explainSys = 1;
				 }
			  } else {		  
				//checking for multi-step unsatisfiability between env transitions and initial condition
				  if (g.calculate_strategy(1, g.getSysPlayer().initial().and(g.getEnvPlayer().initial()), false)) { 
						 debugInfo += "EnvInitTrans UNSAT " + "\n";
						 explainEnv = 1;
				  }
			  }
		  }	catch (Exception e){//Catch exception if any
				System.err.println("Error: " + e.getMessage());
	  }
		  
		  

	
		  if (explainSys ==0 && !sysUnreal.equals(Env.FALSE())) {		 
				 debugInfo += "SysTrans UNREAL"+ "\n"; //TRUE if unsat
		  		 explainSys = 1;
		  }
		  
		  if (explainEnv ==0 && !envUnreal.equals(Env.FALSE())) {		 
			  	debugInfo += "EnvTrans UNREAL"+ "\n"; //TRUE if unsat
		  		explainEnv = 1;
		  }
		  
		  
		  try{
			    debugInfo += justiceChecks(env,sys,explainSys,explainEnv);
			    FileWriter fstream = new FileWriter(filename);
			        BufferedWriter out = new BufferedWriter(fstream);
			    out.append(debugInfo);
			    out.close();
		  }catch (Exception e){//Catch exception if any
			      System.err.println("Error: " + e.getMessage());
		  }
	}
	
	public static class NOPPrintStream extends PrintStream
	{
	    public NOPPrintStream() { super((OutputStream)null); }

	    public void println(String s) { /* Do nothing */ }
	    // You may or may not have to override other methods
	}
	
	public static String justiceChecks(SMVModule env, SMVModule sys, int explainSys, int explainEnv) throws GameException  {
		
		BDD all_init = sys.initial().and( env.initial());
		BDD counter_exmple;
		GROneGame g;
		
		String debugInfo = "";
		BDD prev;
		g = new GROneGame(env,sys, sys.justiceNum(), env.justiceNum());
			
		counter_exmple = g.envWinningStates().and(all_init);		 
		if (counter_exmple.isZero()) {			
			debugInfo += "REALIZABLE\n";
		}	
			
		if (!env.justiceAt(env.justiceNum() - 1).equals(Env.TRUE())) {
		
			
			for (int i = env.justiceNum(); i >=1; i--){		
				//checking for unrealizable environment justice 				
				if (env.justiceAt(i-1).isZero()) {
					debugInfo += "EnvGoals UNSAT " + (i-1) + "\n";
					explainEnv = 1;
				}
				else if ((env.trans().and(Env.prime(env.justiceAt(i-1))).isZero()) | (env.trans().and(env.justiceAt(i-1))).isZero()) { 
					debugInfo += "EnvGoalsTrans UNSAT " + (i-1) + "\n";
					 explainEnv = 1;
				}
				 prev = counter_exmple;
				 g = new GROneGame(env,sys, sys.justiceNum(), i);
				 counter_exmple = g.envWinningStates().and(all_init);
				 if (explainEnv ==0) {
					 if (g.calculate_strategy(3, all_init.and(g.sysWinningStates()), false)) { 
						 //checking for multi-step unsatisfiability between env transitions and goals
						 debugInfo += "EnvGoalsTrans UNSAT " + (i-1) + "\n";
						 explainEnv = 1;
						 i = 0;
					 //} else if (counter_exmple.isZero() && !env.justiceAt(i-1).equals(Env.TRUE())) {// && (!prev.isZero())) {
					 } else if (counter_exmple.isZero() && i > 1) {
						 //if we get here, the env is unrealizable because of the current goal
						 debugInfo += "EnvGoals UNREAL " + (i-1) + "\n";	
						 explainEnv = 1;
						 i = 0;
					 }
				 } 
			}
		}
		
		try {
			while (env.justiceNum() > 0) {
				env.popLastJustice();
			}
			env.addJustice(Env.TRUE());
		}	catch (Exception e){//Catch exception if any
			System.err.println("Error: " + e.getMessage());
		}
		
		if (!sys.justiceAt(0).equals(Env.TRUE())) {
			for (int i = 1; i <= sys.justiceNum(); i++){
				 if (sys.justiceAt(i-1).isZero()) {
					 debugInfo += "SysGoals UNSAT " + (i-1) + "\n";
					 explainSys = 1;
				 }
				 else if ((sys.trans().and(Env.prime(sys.justiceAt(i-1))).isZero()) | sys.trans().and(sys.justiceAt(i-1)).isZero()) {
					 debugInfo += "SysGoalsTrans UNSAT " + (i-1) + "\n";	
				 	 explainSys = 1;
				 }
			 
				 g = new GROneGame(env,sys, i, 1);
				 counter_exmple = g.envWinningStates().and(all_init);
				 if (explainSys ==0 && !counter_exmple.isZero()) {
					//checking for multi-step unsatisfiability between sys transitions and goals
					 if (g.calculate_counterstrategy(counter_exmple, true, false)) {
						 debugInfo += "SysGoalsTrans UNSAT " + (i-1) + "\n";
						 explainSys = 1;
					 } else {//&& (!sys.justiceAt(i-1).equals(Env.TRUE()))) {
						//if we get here, the sys is unrealizable because of the current goal
						 debugInfo += "SysGoals UNREAL " + (i-1) + "\n";
						 explainSys = 1;		
					 }
					 i = sys.justiceNum() + 1;
				 }
			}
		}
		return debugInfo;
	}
}