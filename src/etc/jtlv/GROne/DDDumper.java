import net.sf.javabdd.BDD;
import net.sf.javabdd.BDDFactory;
import net.sf.javabdd.BDDDomain;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.Writer;
import java.io.IOException;
import java.util.*;

public class DDDumper {
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

    /*** Dumping-to-file functions ***/

    public static void writeStrategyBDDToFile(GROneGame g, String filename) throws IOException {
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
        newOrdering = oldOrdering; // for now, reordering is a no-op (TODO: enable this, after also implementing
                                   // on Python side)
        f.setVarOrder(newOrdering);

        // Flatten all the BDDs
        for (i = 0; i < g.sysJustNum; i++) {
            flat_strat.orWith(strat[0][i].and(jx_domain.ithVar(i)).and(strat_type_domain.ithVar(0)));
            flat_strat.orWith(strat[1][i].and(jx_domain.ithVar(i)).and(strat_type_domain.ithVar(1)));
        }

        String[] additional_comments = new String[] {
            "This BDD is a strategy.",
            "",
            "This header contains extra information used by LTLMoP's BDDStrategy.",
            "Currently, the only metadata is 1) the total number of system goals",
            "and 2) the mapping between variable numbers and proposition names.",
            "",
            "Some special variables are also added:",
            "\t- `_jx_b*` are used as a binary vector (b0 is LSB) to indicate",
            "\t  the index of the currently-pursued goal.",
            "\t- `strat_type` is a binary variable used to indicate whether we are",
            "\t   moving closer to the current goal (0) or transitioning to the next goal (1)",
            "",
            "Num goals: " + g.sysJustNum
        };

        writeBDDToFile(flat_strat, filename, additional_comments);
    }

    private static String getEffectiveVarName(BDD bdd, int var_num) {
        // Convert a variable index into a variable name, expanding domains
        // in a way compatible with LTLMoP in the process.

        BDDFactory f = bdd.getFactory();

        String var_name = f.ithVar(var_num).support().toString();

        // Strip the leading and trailing angle brackets
        var_name = var_name.replaceAll("<", "");
        var_name = var_name.replaceAll(">", "");

        // See if this var is part of any domain
        for (int domain_idx = 0; domain_idx < f.numberOfDomains(); domain_idx++) {
            BDDDomain this_domain = f.getDomain(domain_idx);

            // Single variables are actually considered domains too,
            // but we don't care about those
            if (this_domain.varNum() == 1) {
                continue;
            }

            // Detect if var_num is inside this_domain
            // Note: this code assumes that BDDDomain.vars() is ordered
            int[] domain_var_nums = this_domain.vars();
            for (int idx = 0; idx < domain_var_nums.length; idx++) {
                if (domain_var_nums[idx] == var_num) {
                    // Add on a _bX suffix for var names in domains
                    var_name += "_b" + Integer.toString(idx);
                    break;
                }
            }
        }

        return var_name;
    }

    public static void writeBDDToFile(BDD bdd, String filename, String[] additional_comments) throws IOException {
        BDDFactory f = bdd.getFactory();

        // Open up the output file
        LinewiseBufferedWriter out_writer = new LinewiseBufferedWriter(new FileWriter(filename));

        /*** Print header information ***/
        out_writer.write("# This file is a BDD exported by the LTLMoP toolkit.\n");
        out_writer.write("#\n");

        // Stick any extra comments in
        for (String this_line : additional_comments) {
            out_writer.write("# " + this_line + "\n");
        }

        out_writer.write("# Variable names: \n");

        // Output mapping between variable numbers and names
        for (int i = 0; i < f.varNum(); i++) {
            out_writer.write("#\t" + i + ": " + getEffectiveVarName(bdd, i) + "\n");
        }

        out_writer.write("#\n");
        out_writer.write("# For information about the DDDMP format, please see:\n");
        out_writer.write("#    http://www.cs.uleth.ca/~rice/cudd_docs/dddmp/dddmpAllFile.html#dddmpDump.c\n");
        out_writer.write("#\n");
        out_writer.write("# For information about how this file is generated, please see the LTLMoP source:\n");
        out_writer.write("#    src/etc/jtlv/GROne/DDDumper.java\n");
        out_writer.write("#\n");
        out_writer.write(".ver DDDMP-2.0\n");
        out_writer.write(".add\n"); // We are using a 0-1 ADD because dddmp requires BDDs to be in Canonical Complementary Form
        out_writer.write(".mode A\n"); // TODO: use binary for storage efficiency?
        out_writer.write(".varinfo 4\n"); // not totally sure what this means, but is DDDMP_VARDEFAULT
        out_writer.write(".nnodes " + (bdd.nodeCount() + 2) + "\n"); // +2 for leaf nodes
        out_writer.write(".nvars " + f.varNum() + "\n");
        out_writer.write(".nsuppvars " + f.varNum() + "\n"); // TODO: what does this do? will we have a problem if there is a BDD where
                                                             // one or more vars is never used in a node?
        //out_writer.write(".nsuppvars " + countNonzeroEntries(flat_strat.varProfile()) + "\n"); // TODO: what does this do?
        out_writer.write(".ids " + joinIntsWithSpaces(f.getVarOrder()) + "\n"); // TODO: what if it was already re-ordered?
        out_writer.write(".permids ");
        for (int i = 0; i < f.varNum(); i++) {
            out_writer.write(f.var2Level(i) + " ");
        }
        out_writer.write("\n");
        out_writer.write(".nroots 1\n");
        out_writer.write(".rootids " + (bdd.nodeCount() + 2) + "\n"); // last node is root

        // Dump the BDD itself
        out_writer.write(".nodes\n");
        f.save((BufferedWriter)out_writer, bdd);
        out_writer.write(".end\n");

        out_writer.flush();
        out_writer.close();
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
