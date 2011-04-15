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
		 
		 
		 /*String valid = "";
		 
		 try{			 
			 LTLModelCheckAlg alg = new LTLModelCheckAlg(sys, new SpecBDD(Env.TRUE()));
			 //AlgRunnerThread thread = new AlgRunnerThread(alg);
			 //debugInfo+="*** Property is VALID ***";
			 //AlgResultI res1 = alg.preAlgorithm();
			 //debugInfo+=res1.resultString();//alg.preAlgorithm();
			 AlgResultI res = alg.doAlgorithm();			 
			 debugInfo+=res.resultString();//alg.preAlgorithm();
			 //debugInfo+="grr\n";
			 //debugInfo+=thread.getPreResult();
		 }catch (Exception e){//Catch exception if any
		      System.err.println("Error: " + e.getMessage());
		      debugInfo += e.getMessage();
		 }*/

		 if (sys.initial().isZero()) {
			 //debugInfo += "Unsat initial conditions." + "\n";
			 debugInfo += "SysInit UNSAT" + "\n";
			 explainSys = 1;
		 
		 } else if ((sys.initial().and(sys.trans())).isZero()) {
			 //debugInfo += "Unsat transitions." + "\n";
			 debugInfo += "SysTrans UNSAT" + "\n";
			 explainSys = 1;
		 } else {
			// String valid = (new LTLValidAlg(sys, (new SpecBDD(Env.TRUE())))).doAlgorithm().resultString();
			 //debugInfo += "Unsat transitions." + "\n";
			 //if (valid.equals()) {
			 //debugInfo += valid;
			 //explainSys = 1;
			 //}
		  
		 //} else {
		 	 
			 for (int i = 0; i < sys.justiceNum(); i++) {
				 if (sys.justiceAt(i).isZero()) {
					 //debugInfo += "Unsat goal number " + i + "\n";
					 debugInfo += "SysGoals UNSAT " + i + "\n";
					 explainSys = 1;
				 }
				 else if ((sys.trans().and(Env.prime(sys.justiceAt(i))).isZero()) | sys.trans().and(sys.justiceAt(i)).isZero()) {
					 //debugInfo += "Unsat between trans and goal number " + i + "\n";
				 	 debugInfo += "SysGoalsTrans UNSAT " + i + "\n";	
				 	explainSys = 1;
				 }
			 }
			 			 
		 }
		  
		 if (env.initial().isZero()) {
			  //debugInfo += "REV Unsat initial conditions." + "\n";
			  debugInfo += "EnvInit UNSAT" + "\n";
			  explainEnv = 1;	 
		 } else if ((env.initial().and(env.trans())).isZero()) {
			  //debugInfo += "REV Unsat transitions." + "\n";
			  debugInfo += "EnvTrans UNSAT" + "\n";
			  explainEnv = 1;
		  } else {
				 	 
				 for (int i = 0; i < env.justiceNum(); i++) {
					 if (env.justiceAt(i).isZero()) {
						 //debugInfo += "REV Unsat goal number " + i + "\n";
						 debugInfo += "EnvGoals UNSAT " + i + "\n";
						 explainEnv = 1;
					 }
					 else if ((env.trans().and(Env.prime(env.justiceAt(i))).isZero()) | (env.trans().and(env.justiceAt(i))).isZero()) { 
						 //debugInfo += "REV Unsat between trans and goal number " + i + "\n";
						 debugInfo += "EnvGoalsTrans UNSAT " + i + "\n";
						 explainEnv = 1;
					 }
					
				 }
				 			 
		  }
		  
		  if (explainSys ==0 && !sysUnreal.equals(Env.FALSE())) {		 
				 //debugInfo += "cox Y REV = "+ init_cox_Rev + "\n"; //TRUE if unsat
				 debugInfo += "SysTrans UNREAL"+ "\n"; //TRUE if unsat
		  		 explainSys = 1;
		  }
		  
		  if (explainEnv ==0 && !envUnreal.equals(Env.FALSE())) {		 
			  	//debugInfo += "cox Y = "+ init_cox + "\n"; //TRUE if unsat
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
		
		if (!sys.justiceAt(sys.justiceNum() - 1).equals(Env.TRUE())) {
		
			for (int i = 1; i <= sys.justiceNum(); i++){
				 g = new GROneGame(env,sys, i, env.justiceNum());
				 counter_exmple = g.envWinningStates().and(all_init);
				 if (explainSys ==0 && !counter_exmple.isZero()) {//&& (!sys.justiceAt(i-1).equals(Env.TRUE()))) {
					 //debugInfo += "sl Y = "+ (i-1) + "\n";
					 //debugInfo += "System is unrealizable because of justice "+ (i-1) + "\n";
					 debugInfo += "SysGoals UNREAL " + (i-1) + "\n";
					 i = sys.justiceNum() + 1;
				 }
			}
		}
		
		if (!env.justiceAt(env.justiceNum() - 1).equals(Env.TRUE())) {
			
			BDD prev;
			g = new GROneGame(env,sys, sys.justiceNum(), env.justiceNum());
			
			counter_exmple = g.envWinningStates().and(all_init);		 
			if (counter_exmple.isZero()) {			
				debugInfo += "REALIZABLE\n";
			}	
			for (int i = env.justiceNum()-1; i >=1; i--){		
				 prev = counter_exmple;
				 g = new GROneGame(env,sys, sys.justiceNum(), i);
				 counter_exmple = g.envWinningStates().and(all_init);
				 if (explainEnv ==0 && counter_exmple.isZero() && (!env.justiceAt(i-1).equals(Env.TRUE()))) {// && (!prev.isZero())) {
					 //debugInfo += "sl Y REV = "+ (i-1) + "\n";
					 //debugInfo += "Environment is unrealizable because of justice "+ (i-1) + "\n";
					 debugInfo += "EnvGoals UNREAL " + (i) + "\n";				 
					 i = 0;
				 } 
			}
		}
		return debugInfo;
	}
}
