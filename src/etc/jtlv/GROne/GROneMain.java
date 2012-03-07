import net.sf.javabdd.BDD;
import net.sf.javabdd.BDDVarSet;
import net.sf.javabdd.BDD.BDDIterator;
import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.SMVModule;
import edu.wis.jtlv.env.spec.Spec;
import java.io.File;
import java.io.PrintStream;
import java.io.BufferedWriter;
import java.io.FileWriter;
import edu.wis.jtlv.lib.FixPoint;

public class GROneMain {

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

		// env.setFullPrintingMode(true);
		// System.out.println(env);
		// sys.setFullPrintingMode(true);
		// System.out.println(sys);

		// ***** playing the game

		System.out.print("==== Constructing and playing the game ======\n");
		long time = System.currentTimeMillis();
		
		//First try fastslow	
		
		GROneGame g = new GROneGame(env,sys,true);
		long t1 = (System.currentTimeMillis() - time);
		System.out.println("Games time: " + t1);
		
		//Check that every initial system state is winning for every initial environment state
		 BDD all_init = g.getSysPlayer().initial().and(g.getEnvPlayer().initial());
		 BDD counter_exmple = g.envWinningStates().and(all_init);
		 if (!counter_exmple.isZero()) {
			 System.out.println("Specification is unsynthesizable for slow and fast actions...");
			 System.out.println("The env player can win from states:");
			 System.out.println("\t" + counter_exmple);
			 

			 g = new GROneGame(env,sys);
			 t1 = (System.currentTimeMillis() - time);
			 System.out.println("Games time: " + t1);
			
	        // ** Export safety automaton for counterstrategy visualization
			
	        if (args.length == 3 && args[2].equals("--safety")) {
	            System.out.println("Exporting safety constraints automaton...");
	            PrintStream orig_out = System.out;
	            String safety_filename = args[1].replaceAll("\\.[^\\.]+$","_safety.aut");
	            System.setOut(new PrintStream(new File(safety_filename))); // writing the output to a file
	            g.generate_safety_aut(g.getEnvPlayer().initial().and(
	                                  g.getSysPlayer().initial()));
	            System.setOut(orig_out); // restore STDOUT
	            //return;
	        }
	
	        // ** Analysis calls
	
			String debugFile = args[1].replaceAll("\\.[^\\.]+$",".debug");
			GROneDebug.analyze(env,sys);
	
			 ///////////////////////////////////////////////
			 //Check that every initial system state is winning for every initial environment state
			 all_init = g.getSysPlayer().initial().and(g.getEnvPlayer().initial());
			 counter_exmple = g.envWinningStates().and(all_init);
			 if (!counter_exmple.isZero()) {
				 System.out.println("Specification is unsynthesizable even assuming instantaneous actions...");
				 System.out.println("The env player can win from states:");
				 System.out.println("\t" + counter_exmple);
				 
				 
				 
				// If you only care about the existence of a winning initial system state for every initial environment state
					// (vs. every initial system state being winning for every initial environment state)
				/*	BDD env_ini = g.getEnvPlayer().initial();
					BDDVarSet env_vars = g.getEnvPlayer().moduleUnprimeVars();
					for (BDDIterator it = env_ini.iterator(env_vars); it.hasNext();) {
						BDD eini = (BDD) it.next();
						BDD sys_response = eini.and(g.getSysPlayer().initial()).and(
								g.sysWinningStates());
			            System.out.println("---------------");
			            sys_response.printSet();
						if (sys_response.isZero()) {
							System.out.println("Specification is unrealizable...");
							System.out.println("The env player can win from states:");
							System.out.println("\t" + eini);
							System.out.println("===== Done ==============================");
							return;
						}
				}*/
				 
				 	
				 System.out.println("==== Computing counterstrategy =========");
				 System.out.println("-----------------------------------------");
				 PrintStream orig_out = System.out;
				 System.setOut(new PrintStream(new File(out_filename))); // writing the output to a file
				 g.printLosingStrategy(counter_exmple);
				 System.setOut(orig_out); // restore STDOUT
				 System.out.print("-----------------------------------------\n");
				 long t2 = (System.currentTimeMillis() - time);
				 System.out.println("Strategy time: " + t2);
				 System.out.println("===== Done ==============================");
					
				
			     //Error code = 1 on exit
				 System.exit(1);
			 }
	
		
			System.out.println("Specification is realizable assuming instantaneous actions...");
			System.out.println("==== Building an implementation =========");
			System.out.println("-----------------------------------------");
			PrintStream orig_out = System.out;
			System.setOut(new PrintStream(new File(out_filename))); // writing the output to a file
			g.printWinningStrategy(all_init);
			System.setOut(orig_out); // restore STDOUT
			System.out.print("-----------------------------------------\n");
			long t2 = (System.currentTimeMillis() - time);
			System.out.println("Strategy time: " + t2);
			System.out.println("===== Done ==============================");
			System.exit(0);
		 }
		 
		 System.out.println("Specification is realizable with slow and fast actions...");
		 System.out.println("==== Building an implementation =========");
		 System.out.println("-----------------------------------------");
		 PrintStream orig_out = System.out;
		 System.setOut(new PrintStream(new File(out_filename))); // writing the output to a file
		 g.printWinningStrategy(all_init);
		 System.setOut(orig_out); // restore STDOUT
		 System.out.print("-----------------------------------------\n");
		 long t2 = (System.currentTimeMillis() - time);
		 System.out.println("Strategy time: " + t2);
		 System.out.println("===== Done ==============================");
		 System.exit(0);	
	}
	
	
	
}
