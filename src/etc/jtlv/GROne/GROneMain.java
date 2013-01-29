import edu.wis.jtlv.env.module.ModuleBDDField;
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
            System.err.println("Usage: java GROneMain <smv_file> <ltl_file> [--fastslow] [--safety]");
            System.exit(1);
        }                

        // Load SMV and LTL files
        Env.loadModule(args[0]);
        Spec[] spcs = Env.loadSpecFile(args[1]);
        
        // Parse extra command-line switches
        boolean fs = false;
        boolean gen_safety = false;

        for (int i = 2; i < args.length; i++) {
            if (args[i].equals("--fastslow")) {
                fs = true;
            } else if (args[i].equals("--safety")) {
                gen_safety = true;
            } else {
                System.err.println("Unknown option: " + args[i]);
                System.err.println("Usage: java GROneMain <smv_file> <ltl_file> [--fastslow] [--safety]");
                System.exit(1);
            }
        }

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
	
		GROneGame g;
		BDD all_init, counter_exmple;
		
		//If the option is enabled, first try fastslow	
		if (fs) {
			g = new GROneGame(env,sys,true);
			long t1 = (System.currentTimeMillis() - time);
			System.out.println("Games time: " + t1);
			
			//Check that every initial system state is winning for every initial environment state
			 all_init = g.getSysPlayer().initial().and(g.getEnvPlayer().initial());
			 counter_exmple = g.envWinningStates().and(all_init);
			 if (counter_exmple.isZero()) {
				 
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
			 else {
				 System.out.println("Specification is unsynthesizable for slow and fast actions...");
				 System.out.println("The env player can win from states:");
				 System.out.println("\t" + counter_exmple);
			 }
		}
		

		g = new GROneGame(env,sys);
		long t3 = (System.currentTimeMillis() - time);
		System.out.println("Games time: " + t3);

		// ** Export safety automaton for counterstrategy visualization

		if (gen_safety) {
			System.out.println("Exporting safety constraints automaton...");
			PrintStream orig_out = System.out;
			String safety_filename = args[1].replaceAll("\\.[^\\.]+$","_safety.aut");
			System.setOut(new PrintStream(new File(safety_filename))); // writing the output to a file
			g.generate_safety_aut(g.getEnvPlayer().initial().and(
					g.getSysPlayer().initial()));
			System.setOut(orig_out); // restore STDOUT
			//return;
		}

		

		///////////////////////////////////////////////
		//Check that every initial system state is winning for every initial environment state
		all_init = g.getSysPlayer().initial().and(g.getEnvPlayer().initial());
		counter_exmple = g.envWinningStates().and(all_init);
        BDDVarSet all_vars = sys.moduleUnprimeVars().union(env.moduleUnprimeVars());
		if (!counter_exmple.isZero()) {
			System.out.println("Specification is unsynthesizable even assuming instantaneous actions...");
			System.out.println("The env player can win from " + (int)counter_exmple.satCount(all_vars) + " of " + (int)all_init.satCount(all_vars) +" initial state(s).");
            String counter_state = counter_exmple.satOne().toString();
            counter_state = counter_state.replaceAll("and", "&");
            counter_state = counter_state.replaceAll("[()]", "");
            counter_state = counter_state.replaceAll("main\\.([\\w.]+)=0", "!$1");
            counter_state = counter_state.replaceAll("main\\.([\\w.]+)=1", "$1");
			System.out.println("\tFor example: \t" + counter_state);



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
		boolean rho3 = g.printWinningStrategy(all_init);
		System.setOut(orig_out); // restore STDOUT
		System.out.print("-----------------------------------------\n");
		long t2 = (System.currentTimeMillis() - time);
		System.out.println("Strategy time: " + t2);	
		if (rho3) {
			System.out.println("Environment liveness falsified"); 
		}
		System.out.println("===== Done ==============================");
		
		// ** Analysis calls

				String debugFile = args[1].replaceAll("\\.[^\\.]+$",".debug");
				GROneDebug.analyze(env,sys);
				
		System.exit(0);
		
		
	
	}
}
