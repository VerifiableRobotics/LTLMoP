import edu.wis.jtlv.env.module.ModuleBDDField;
import net.sf.javabdd.BDD;
import net.sf.javabdd.BDDVarSet;
import net.sf.javabdd.BDDFactory;
import net.sf.javabdd.BDDDomain;
import net.sf.javabdd.BDD.BDDIterator;
import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.SMVModule;
import edu.wis.jtlv.env.spec.Spec;
import java.io.File;
import java.io.PrintStream;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.Writer;
import java.io.IOException;
import edu.wis.jtlv.lib.FixPoint;
import java.util.*;

/** 
 * GROneMain is the main front-end to the synthesis subsystem
 */

public class GROneMain {
    /*** Extremely mundane helper functions (aka, a reminder that Java is not Python) ***/

    private static int countNonzeroEntries(int[] array) {
        int count = 0;
        for (int item : array) {
            if (item != 0) {
                count++;
            }
        }
        return count;
    }  
    
    private static String joinIntsWithSpaces(int[] nums) {
        // return empty string for empty list
        if (nums.length == 0)
            return "";
            
        String s = Integer.toString(nums[0]);
        for (int i = 1; i < nums.length; i++) {
            s += " " + nums[i];
        }
        return s;
    }

    /*** Helper functions ***/

    private static void printUsage() {
        // Print command-line usage information
        System.err.println("Usage: java GROneMain <smv_file> <ltl_file> [--fastslow] [--safety] [--symbolic]");
        System.err.println("");
        System.err.println("Options: --fastslow (enable extra checks for safety of implicit intermediate states during synthesis)");
        System.err.println("         --safety   (in addition to strategy, export system transition relations)");
        System.err.println("         --symbolic (output BDDs instead of explicit-state automata)");
    }

    private static boolean synthesisSucceeded(GROneGame g) {
        // Return True iff the system can win from all initial states

        BDD all_init, counter_example;

        all_init = g.getSysPlayer().initial().and(g.getEnvPlayer().initial());
        counter_example = g.envWinningStates().and(all_init);

        return counter_example.isZero();
           
        // If you only care about the existence of a winning initial system state for every initial environment state
        // (vs. every initial system state being winning for every initial environment state):
        /*	
        BDD env_ini = g.getEnvPlayer().initial();
            BDDVarSet env_vars = g.getEnvPlayer().moduleUnprimeVars();
            for (BDDIterator it = env_ini.iterator(env_vars); it.hasNext();) {
                BDD eini = (BDD) it.next();
                BDD sys_response = eini.and(g.getSysPlayer().initial()).and(
                        g.sysWinningStates());
                if (sys_response.isZero()) {
                    return false;
                }
        }
        return true;
        */
    }

    /*** Main routine ***/

	public static void main(String[] args) throws Exception {
		// uncomment to use a C BDD package
		//System.setProperty("bdd", "buddy");

		GROneParser.just_initial = true; // what is this for?

        // Check that we have enough arguments
        if (args.length < 2) {
            printUsage();
            System.exit(1);
        }                

        /* Parse extra command-line switches */

        // defaults
        boolean opt_fs = false;
        boolean opt_gen_safety = false;
        boolean opt_symbolic = false;

        for (int i = 2; i < args.length; i++) {
            if (args[i].equals("--fastslow")) {
                opt_fs = true;
            } else if (args[i].equals("--safety")) {
                opt_gen_safety = true;
            } else if (args[i].equals("--symbolic")) {
                opt_symbolic = true;
            } else {
                System.err.println("Unknown option: " + args[i]);
                printUsage();
                System.exit(1);
            }
        }

        // Load SMV and LTL files
        Env.loadModule(args[0]);
        Spec[] spcs = Env.loadSpecFile(args[1]);
        
        // Figure out the base name for our output files by stripping the .smv filename extension
        String out_filename_base = args[1].replaceAll("\\.[^\\.]+$","");
        String out_filename;

		// Construct the environment module.
		SMVModule env = (SMVModule) Env.getModule("main.e");
		Spec[] env_conjuncts = GROneParser.parseConjuncts(spcs[0]);
		GROneParser.addReactiveBehavior(env, env_conjuncts);

		// Construct the system module.
		SMVModule sys = (SMVModule) Env.getModule("main.s");
		Spec[] sys_conjuncts = GROneParser.parseConjuncts(spcs[1]);
		GROneParser.addReactiveBehavior(sys, sys_conjuncts);

		// env.setFullPrintingMode(true);
		// System.out.println(env);
		// sys.setFullPrintingMode(true);
		// System.out.println(sys);

		// ** Play the game

		System.out.print("====> Checking synthesizability...\n");
	
        long tic, toc;
		GROneGame g = null;
		
		// If the option is enabled, first try fastslow	
		if (opt_fs) {
            tic = System.currentTimeMillis();		
			g = new GROneGame(env,sys,true);
		    toc = System.currentTimeMillis();
			System.out.println("Completed in: " + (toc-tic)/1000.0 + "sec");
			
            if (synthesisSucceeded(g)) {
                 System.out.println("Specification is synthesizable under fast/slow!");
            } else {
		     	 System.out.println("Unsynthesizable under fast/slow.  Falling back to normal...");
            }
        }

        // If fastslow failed or wasn't chosen, synthesize normally
        if (!opt_fs || !synthesisSucceeded(g)) {
            tic = System.currentTimeMillis();		
			g = new GROneGame(env,sys);
		    toc = System.currentTimeMillis();
			System.out.println("Completed in: " + (toc-tic)/1000.0 + "sec");
        }

		// ** Output a strategy, if one exists

        if (synthesisSucceeded(g)) {
            System.out.println("Specification is synthesizable!");
            System.out.println("====> Building an implementation...");

            out_filename = out_filename_base + (opt_symbolic ? ".bdd" : ".aut");

            tic = System.currentTimeMillis();		

            if (opt_symbolic) {
                // Calculate the strategy
                BDD[][] strat = g.calculate_symb_strategy();

                /** Note: in the following, we combine all of the strategy BDDs into a single BDD.
                 *  This is mostly just so we can easily write to a file, since BuDDy doesn't seem
                 *  to support BDD array storage; it shouldn't increase the size significantly as long as
                 *  the ordering is kept reasonable.
                 */

                BDDFactory f = strat[0][0].getFactory(); 
                BDD flat_strat = f.zero();

                // Construct domains for jx and strat_type
                BDDDomain jx_domain = f.extDomain(g.sysJustNum);
                jx_domain.setName("_jx");
                BDDDomain strat_type_domain = f.extDomain(2);
                strat_type_domain.setName("strat_type");

                // Make sure these domains are first in the ordering
                int[] oldOrdering = f.getVarOrder();
                int[] newOrdering = new int[oldOrdering.length];

                // TODO: which makes more sense to have first?
                int i = 0;
                for (int v : jx_domain.vars()) { 
                    newOrdering[i] = v;
                    i++;
                }
                for (int v : strat_type_domain.vars()) { 
                    newOrdering[i] = v;
                    i++;
                }
                for (int v : oldOrdering) {
                    boolean alreadyInOrdering = false;
                    for (int j = 0; j < i; j++) {
                        // omg i miss python
                        if (newOrdering[j] == v) {
                            alreadyInOrdering = true;
                            break;
                        }
                    }
                    if (!alreadyInOrdering) {
                        newOrdering[i] = v;
                        i++;
                    }
                }

                // Re-order for storage efficiency
                f.reorderVerbose(10);
                newOrdering = oldOrdering; // for now, reordering is a no-op (TODO: enable this)
                f.setVarOrder(newOrdering);

                // Flatten all the BDDs
                for (i = 0; i < g.sysJustNum; i++) {
                    //jx_domain.ithVar(i).printSet();
                    flat_strat.orWith(strat[0][i].and(jx_domain.ithVar(i)).and(strat_type_domain.ithVar(0)));
                    flat_strat.orWith(strat[1][i].and(jx_domain.ithVar(i)).and(strat_type_domain.ithVar(1)));
                } 

                // Open up the output file
                LinewiseBufferedWriter out_writer = new LinewiseBufferedWriter(new FileWriter(out_filename));
    
                // Print header information
                out_writer.write("# This is a strategy BDD generated by the LTLMoP toolkit.\n");
                out_writer.write("#\n");
                out_writer.write("# TODO: explain format here\n");
                out_writer.write("# Max goal ID: " + g.sysJustNum + "\n");
                out_writer.write("# Variable names: \n");
                for (i=0 ; i<newOrdering.length; i++) {
                    String var_name = f.ithVar(i).support().toString();
                    // Strip the leading and trailing angle brackets
                    var_name = var_name.replaceAll("<", "");
                    var_name = var_name.replaceAll(">", "");
                    // manually add on the _bX for jx domain
                    int idx = i - jx_domain.vars()[0];
                    if (idx >= 0 && idx < jx_domain.varNum()) {
                        var_name += "_b" + Integer.toString(idx);
                    }
                    out_writer.write("#\t" + i + ": " + var_name + "\n");
                }
                out_writer.write("#\n");
                out_writer.write("# For information about the DDDMP format, please see:\n");
                out_writer.write("#    http://www.cs.uleth.ca/~rice/cudd_docs/dddmp/dddmpAllFile.html#dddmpDump.c\n");
                out_writer.write(".ver DDDMP-2.0\n");
                out_writer.write(".add\n"); // We are using a 0-1 ADD because dddmp requires BDDs to be in Canonical Complementary Form
                out_writer.write(".mode A\n"); // TODO: use binary for storage efficiency?
                out_writer.write(".varinfo 4\n"); // not totally sure what this means, but is DDDMP_VARDEFAULT
                out_writer.write(".nnodes " + (flat_strat.nodeCount() + 2) + "\n"); // +2 for leaf nodes
                out_writer.write(".nvars " + newOrdering.length + "\n");
                out_writer.write(".nsuppvars " + newOrdering.length + "\n"); // TODO: what does this do?
                //out_writer.write(".nsuppvars " + countNonzeroEntries(flat_strat.varProfile()) + "\n"); // TODO: what does this do?
                out_writer.write(".ids " + joinIntsWithSpaces(oldOrdering) + "\n"); // TODO: what if it was already re-ordered?
                out_writer.write(".permids ");
                for (i=0 ; i<oldOrdering.length; i++) {
                    out_writer.write(f.var2Level(i) + " ");
                }
                out_writer.write("\n");
                out_writer.write(".nroots 1\n");
                out_writer.write(".rootids " + (flat_strat.nodeCount() + 2) + "\n");

                // Dump the BDD itself
                out_writer.write(".nodes\n");
                f.save((BufferedWriter)out_writer, flat_strat);
                out_writer.write(".end\n");

                out_writer.flush();
                out_writer.close();
            } else {
                PrintStream orig_out = System.out;

                System.setOut(new PrintStream(new File(out_filename))); // redirect the output to a file

                BDD all_init = g.getSysPlayer().initial().and(g.getEnvPlayer().initial());
                boolean falsifyEnv = g.printWinningStrategy(all_init);
				if (falsifyEnv) {
					System.out.println("WARNING: Environment liveness falsified"); 
				}

                System.setOut(orig_out); // restore STDOUT
            }

            toc = System.currentTimeMillis();		

            System.out.println("Completed in: " + (toc-tic)/1000.0 + "sec");

            System.out.println("Strategy written to file: " + out_filename);

        } else {

            System.out.println("ERROR: Specification is unsynthesizable.");

			/* Give some basic feedback about losing initial states */
			BDD all_init = g.getSysPlayer().initial().and(g.getEnvPlayer().initial());
			BDD counter_example = g.envWinningStates().and(all_init);
        	BDDVarSet all_vars = sys.moduleUnprimeVars().union(env.moduleUnprimeVars());
		
        	System.out.println("The env player can win from " + (int)counter_example.satCount(all_vars) + " of " + (int)all_init.satCount(all_vars) +" initial state(s).");
            String counter_state = counter_example.satOne().toString();
            counter_state = counter_state.replaceAll("and", "&");
            counter_state = counter_state.replaceAll("[()]", "");
            counter_state = counter_state.replaceAll("main\\.([\\w.]+)=0", "!$1");
            counter_state = counter_state.replaceAll("main\\.([\\w.]+)=1", "$1");
			System.out.println("\tFor example: \t" + counter_state);
        }
		
		// ** Export safety automaton

		if (opt_gen_safety) {
			System.out.println("====> Exporting safety constraints automaton...");

			PrintStream orig_out = System.out;
			out_filename = out_filename_base + "_safety.aut";
            System.setOut(new PrintStream(new File(out_filename))); // redirect the output to a file

            tic = System.currentTimeMillis();		

			g.generate_safety_aut(g.getEnvPlayer().initial().and(
					g.getSysPlayer().initial()));

            toc = System.currentTimeMillis();		

			System.setOut(orig_out); // restore STDOUT

			System.out.println("Completed in: " + (toc-tic)/1000.0 + "sec");
		}

		

        if (!synthesisSucceeded(g)) {
			System.out.println("====> Performing unsynthesizability analysis...");
            tic = System.currentTimeMillis();		
            GROneDebug.analyze(env,sys);
            toc = System.currentTimeMillis();		
			System.out.println("Completed in: " + (toc-tic)/1000.0 + "sec");

			System.out.println("====> Computing counterstrategy...");

            // TODO: make the counterstrat aut have a distinct filename?
            // TODO: enable BDD export of counterstrat too
			out_filename = out_filename_base + ".aut";

            PrintStream orig_out = System.out;
            System.setOut(new PrintStream(new File(out_filename))); // writing the output to a file

            tic = System.currentTimeMillis();		

            BDD all_init, counter_example;

            all_init = g.getSysPlayer().initial().and(g.getEnvPlayer().initial());
            counter_example = g.envWinningStates().and(all_init);
            g.printLosingStrategy(counter_example);

            toc = System.currentTimeMillis();		

            System.setOut(orig_out); // restore STDOUT

			System.out.println("Completed in: " + (toc-tic)/1000.0 + "sec");
        }

        //return !synthesisSucceeded(g);
	}

    static class LinewiseBufferedWriter extends BufferedWriter {
        /** This class is an attempt to deal with the fact that dddmp requires sequential
         *  numbering of nodes, starting with 1, while the JTLV export function
         *  prints node names that are effectively arbitrary.  To avoid memory problems,
         *  we rewrite the BDD output line-by-line as we go. 
         */

        private String line_buffer;
        private int num_lines_processed;
        private HashMap<Integer,Integer> nodeNameMapping = new HashMap<Integer,Integer>(); // key is old name, value is new name

        public LinewiseBufferedWriter(Writer out) {
            super(out);
            line_buffer = "";
            num_lines_processed = 0;
        }

        private void writeString(String s) {
            try {
                super.write(s, 0, s.length()); 
            } catch (IOException ex) {
                ex.printStackTrace();
            }
        }

        private void processLine(String s) {
            // Don't do any special processing for comments or commands 
            if (s.startsWith(".") || s.startsWith("#")) {
                writeString(s);
                return;
            }

            // Make Node 1 be the True node
            // Make Node 2 be the False node
            if (num_lines_processed == 2) {
                writeString("1 1 0 0\n");
                writeString("2 0 0 0\n");
                nodeNameMapping.put(0, 2);
                nodeNameMapping.put(1, 1);
            }

            // skip the first two lines, since those are metadata not nodes
            if (num_lines_processed >= 2) {
                Scanner sc = new Scanner(s);
                Integer[] nodeData = {sc.nextInt(), sc.nextInt(), sc.nextInt(), sc.nextInt()};

                // remap all the node IDs, because dddmp requires sequential numbering
                nodeNameMapping.put(nodeData[0], num_lines_processed + 1);
                nodeData[2] = nodeNameMapping.get(nodeData[2]);
                nodeData[3] = nodeNameMapping.get(nodeData[3]);
                
                // NOTE: We swap the if/else nodes to make dddmp-compatible output
                // From the BuDDy docs for bdd_load(): "Each set consists of first the node number, then the variable number and then the low and high nodes."
                // From the dddmp "docs": fscanf(fp, "%d %s %d %d\n", &id, buf, &idT, &idE)
                nodeData[0] = num_lines_processed + 1;
                writeString(nodeData[0] + " " + nodeData[1] + " " +
                            nodeData[3] + " " + nodeData[2] + "\n");
            }

            num_lines_processed++;
        }

        public void write(String s, int off, int len) {
            // TODO: use a circular buffer or something more efficient?
            
            // add to the buffer
            line_buffer += s.substring(off, off+len);
            
            // check to see if we've gotten a newline in this last write
            // TODO: do we need to check for other types of newlines too?
            int newline_loc = line_buffer.indexOf("\n");
            if (newline_loc != -1) {
                // handle this line
                processLine(line_buffer.substring(0, newline_loc+1));
                // remove from the buffer
                line_buffer = line_buffer.substring(newline_loc+1);
            }
        }
    }
}
