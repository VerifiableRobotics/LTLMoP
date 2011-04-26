import java.util.Iterator;
import java.util.Stack;
import java.util.Vector;

import net.sf.javabdd.BDD;
import net.sf.javabdd.BDDVarSet;
import net.sf.javabdd.BDD.BDDIterator;
import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.ModuleWithWeakFairness;
import edu.wis.jtlv.env.module.Module;
import edu.wis.jtlv.lib.FixPoint;
import edu.wis.jtlv.old_lib.games.GameException;

/**
 * <p>
 * Nir Piterman, Amir Pnueli, and Yaniv Sa’ar. Synthesis of Reactive(1) Designs.
 * In VMCAI, pages 364–380, Charleston, SC, Jenuary 2006.
 * </p>
 * <p>
 * To execute, create an object with two Modules, one for the system and the
 * other for the environment, and then just extract the strategy through
 * {@link edu.wis.jtlv.old_lib.games.GR1Game#printWinningStrategy()}.
 * </p>
 * 
 * @version {@value edu.wis.jtlv.env.Env#version}
 * @author yaniv sa'ar. (parts modified by Cameron Finucane)
 * 
 */
public class GROneGame {
	private ModuleWithWeakFairness env;
	private ModuleWithWeakFairness sys;
	int sysJustNum, envJustNum;
	private BDD player1_winning;
	private BDD player2_winning;

	// p2_winning in GRGAmes are !p1_winning

	public GROneGame(ModuleWithWeakFairness env, ModuleWithWeakFairness sys, int sysJustNum, int envJustNum)
	throws GameException {
		if ((env == null) || (sys == null)) {
			throw new GameException(
					"cannot instanciate a GR[1] Game with an empty player.");
		}
		
		this.env = env;
		this.sys = sys;
		this.sysJustNum = sysJustNum;
		this.envJustNum = envJustNum;
		
		// for now I'm giving max_y 50, at the end I'll cut it (and during, I'll
		// extend if needed. (using vectors only makes things more complicate
		// since we cannot instantiate vectors with new vectors)
		x_mem = new BDD[sysJustNum][envJustNum][50];
		y_mem = new BDD[sysJustNum][50];
		z_mem = new BDD[sysJustNum];
		x2_mem = new BDD[sysJustNum][envJustNum][50][50];
		y2_mem = new BDD[sysJustNum][50];
		z2_mem = new BDD[50];	
		this.player2_winning = this.calculate_win();	
		this.player1_winning = this.calculate_loss();
	}
	
	public GROneGame(ModuleWithWeakFairness env, ModuleWithWeakFairness sys)
			throws GameException {
		this(env, sys, sys.justiceNum(), env.justiceNum());
	}

	public BDD[][][] x_mem;
	public BDD[][][][] x2_mem;
	public BDD[][] y_mem, y2_mem;
	public BDD[] z_mem, z2_mem;

	/**
	 * <p>
	 * Calculating winning states.
	 * </p>
	 * 
	 * @return The winning states for this game.
	 */
	private BDD calculate_win() {
		BDD x, y, z;
		FixPoint<BDD> iterZ, iterY, iterX;
		int cy = 0;

		z = Env.TRUE();
		for (iterZ = new FixPoint<BDD>(); iterZ.advance(z);) {
			//for (int j = 0; j < sys.justiceNum(); j++) {
			for (int j = 0; j < sysJustNum; j++) {
				cy = 0;
				y = Env.FALSE();
				for (iterY = new FixPoint<BDD>(); iterY.advance(y);) {
					BDD start = sys.justiceAt(j).and(env.yieldStates(sys, z))
							.or(env.yieldStates(sys, y));
					//y = Env.FALSE();
					for (int i = 0; i < envJustNum; i++) {
						BDD negp = env.justiceAt(i).not();
						x = z.id();
						for (iterX = new FixPoint<BDD>(); iterX.advance(x);) {
							x = negp.and(env.yieldStates(sys, x)).or(start);
						}
						x_mem[j][i][cy] = x.id();
						y = y.id().or(x);
					}
					y_mem[j][cy] = y.id();
					cy++;
					if (cy % 50 == 0) {
						x_mem = extend_size(x_mem, cy);
						y_mem = extend_size(y_mem, cy);
					}
				}
				z = y.id();
				z_mem[j] = z.id();
			}
		}
		x_mem = extend_size(x_mem, 0);
		y_mem = extend_size(y_mem, 0);
		return z.id();
	}
	
	private BDD calculate_loss() {
		BDD x, y, z;
		FixPoint<BDD> iterZ, iterY, iterX;
		int c = 0;
		int a = 0;
		z = Env.FALSE();
		x = Env.FALSE();
		for (iterZ = new FixPoint<BDD>(); iterZ.advance(z);) {
			//for (int j = 0; j < sys.justiceNum(); j++) {
			for (int j = 0; j < sysJustNum; j++) {				
				y = Env.TRUE();
				for (iterY = new FixPoint<BDD>(); iterY.advance(y);) {
					BDD start = ((sys.justiceAt(j).not()).or((env.yieldStates(sys, z.not())).not()))
					//BDD start = ((sys.justiceAt(j).not()).or(sys.yieldStates(sys, z)))
							.and((env.yieldStates(sys, y.not())).not());					
					for (int i = 0; i < envJustNum; i++) {
						x = Env.FALSE();
						c=0;
						for (iterX = new FixPoint<BDD>(); iterX.advance(x);) {
							x = x.id().or((env.justiceAt(i).or(((env.yieldStates(sys, x.not())).not()))).and(start));
							//x = x.id().or((env.justiceAt(i).or(env.yieldStates(sys, x))).and(start));
							x2_mem[j][i][a][c] = x.id();
							c++;
						}
						
						y = y.id().and(x);
					}				
					if (c % 50 == 0) {
						x2_mem = extend_size(x2_mem, c);						
					}
				}

				y2_mem[j][a] = y.id();	
				z = x.id().or(y);				
			}
			
			
			z2_mem[a] = z.id();
			System.out.println("Z = "+z2_mem[a]);
			a++;
			if (a % 50 == 0) {
				z2_mem = extend_size(z2_mem, a);
				y2_mem = extend_size(y2_mem, a);
			}
			
		}
		
		x2_mem = extend_size(x2_mem, 0);
		y2_mem = extend_size(y2_mem, 0);
		z2_mem = extend_size(z2_mem, 0);
		//System.out.println(z.id().equals(player2_winning.not()));
		System.out.println("y2 = "+ y2_mem.length);
		System.out.println("x2 = "+ x2_mem.length);
		return z.id();
	}

	private BDD[][][][] extend_size(BDD[][][][] in, int extended_size) {
		BDD[][][][] res;
		if (extended_size > 0) {
			res = new BDD[in.length][in[0].length][in[0][0].length + extended_size][in[0][0][0].length
					+ extended_size];
			for (int i = 0; i < in.length; i++) {
				for (int j = 0; j < in[i].length; j++) {
					for (int k = 0; k < in[i][j].length; k++) {
						for (int m = 0; m < in[i][j][k].length; m++) {
							res[i][j][k][m] = in[i][j][k][m];
						}
					}
				}
			}
		} else {
			res = new BDD[in.length][in[0].length][in[0][0].length][];
			for (int i = 0; i < in.length; i++) {
				for (int j = 0; j < in[i].length; j++) {
					for (int k = 0; k < in[i][j].length; k++) {						
						int real_size = 0;
						for (int m = 0; m < in[i][j][k].length; m++) {
							if (in[i][j][k][m] != null)
								real_size++;
						}
						res[i][j][k] = new BDD[real_size];
						int new_add = 0;
						for (int m = 0; m < in[i][j][k].length; m++) {
							if (in[i][j][k][m] != null) {
								res[i][j][k][new_add] = in[i][j][k][m];
								new_add++;
							}
						}
					}
				}
			}
		}
		return res;
	}
	
	// extended_size<=0 will tight the arrays to be the exact sizes.
	private BDD[][][] extend_size(BDD[][][] in, int extended_size) {
		BDD[][][] res;
		if (extended_size > 0) {
			res = new BDD[in.length][in[0].length][in[0][0].length
					+ extended_size];
			for (int i = 0; i < in.length; i++) {
				for (int j = 0; j < in[i].length; j++) {
					for (int k = 0; k < in[i][j].length; k++) {
						res[i][j][k] = in[i][j][k];
					}
				}
			}
		} else {
			res = new BDD[in.length][in[0].length][];
			for (int i = 0; i < in.length; i++) {
				for (int j = 0; j < in[i].length; j++) {
					int real_size = 0;
					for (int k = 0; k < in[i][j].length; k++) {
						if (in[i][j][k] != null)
							real_size++;
					}
					res[i][j] = new BDD[real_size];
					int new_add = 0;
					for (int k = 0; k < in[i][j].length; k++) {
						if (in[i][j][k] != null) {
							res[i][j][new_add] = in[i][j][k];
							new_add++;
						}
					}
				}
			}
		}
		return res;
	}

	// extended_size<=0 will tight the arrays to be the exact sizes.
	private BDD[][] extend_size(BDD[][] in, int extended_size) {
		BDD[][] res;
		if (extended_size > 0) {
			res = new BDD[in.length][in[0].length + extended_size];
			for (int i = 0; i < in.length; i++) {
				for (int j = 0; j < in[i].length; j++) {
					res[i][j] = in[i][j];
				}
			}
		} else {
			res = new BDD[in.length][];
			for (int i = 0; i < in.length; i++) {
				int real_size = 0;
				for (int j = 0; j < in[i].length; j++) {
					if (in[i][j] != null)
						real_size++;
				}
				res[i] = new BDD[real_size];
				int new_add = 0;
				for (int j = 0; j < in[i].length; j++) {
					if (in[i][j] != null) {
						res[i][new_add] = in[i][j];
						new_add++;
					}
				}
			}
		}
		return res;
	}
	
	private BDD[] extend_size(BDD[] in, int extended_size) {
		BDD[] res;		
		if (extended_size > 0) {
			res = new BDD[in.length + extended_size];
			for (int i = 0; i < in.length; i++) {
				
					res[i] = in[i];
				
			}
		} else {
				int real_size = 0;
				for (int j = 0; j < in.length; j++) {
					if (in[j] != null)
						real_size++;
				}
				res = new BDD[real_size];
				for (int j = 0; j < real_size; j++) {
					
						res[j] = in[j];
				
				}
		}			
		return res;
	}

	/**
	 * <p>
	 * Extracting an arbitrary implementation from the set of possible
	 * strategies.
	 * </p>
	 */
	public void printWinningStrategy(BDD ini) {
		calculate_strategy(3, ini);
		// return calculate_strategy(3);
		// return calculate_strategy(7);
		// return calculate_strategy(11);
		// return calculate_strategy(15);
		// return calculate_strategy(19);
		// return calculate_strategy(23);
	}
	
	public void printLosingStrategy(BDD ini) {
		calculate_counterstrategy(ini);
		// return calculate_strategy(3);
		// return calculate_strategy(7);
		// return calculate_strategy(11);
		// return calculate_strategy(15);
		// return calculate_strategy(19);
	}
	
	//public void printLosingTrace(BDD ini) {
//		calculate_countertrace(ini);
		// return calculate_strategy(3);
		// return calculate_strategy(7);
		// return calculate_strategy(11);
		// return calculate_strategy(15);
		// return calculate_strategy(19);
//	/}
	
	
	/**
	 * <p>
	 * Extracting an implementation from the set of possible strategies with the
	 * given priority to the next step.
	 * </p>
	 * <p>
	 * Possible priorities are:<br>
	 * 3 - Z Y X.<br>
	 * 7 - Z X Y.<br>
	 * 11 - Y Z X.<br>
	 * 15 - Y X Z.<br>
	 * 19 - X Z Y.<br>
	 * 23 - X Y Z.<br>
	 * </p>
	 * 
	 * @param kind
	 *            The priority kind.
	 */
	public void calculate_strategy(int kind, BDD ini) {
		int strategy_kind = kind;
		Stack<BDD> st_stack = new Stack<BDD>();
		Stack<Integer> j_stack = new Stack<Integer>();
		Stack<RawState> aut = new Stack<RawState>();
		// FDSModule res = new FDSModule("strategy");

        BDDIterator ini_iterator = ini.iterator(env.moduleUnprimeVars().union(sys.moduleUnprimeVars()));

        while (ini_iterator.hasNext()) {

            BDD this_ini = (BDD) ini_iterator.next();

            RawState test_st = new RawState(aut.size(), this_ini, 0);

            int idx = -1;
            for (RawState cmp_st : aut) {
                if (cmp_st.equals(test_st, false)) { // search ignoring rank
                    idx = aut.indexOf(cmp_st);
                    break;
                }
            }

            if (idx != -1) {
                // This initial state is already in the automaton
                continue;
            }

            // Otherwise, we need to attach this initial state to the automaton

            st_stack.push(this_ini);
            j_stack.push(new Integer(0)); // TODO: is there a better default j?
            this_ini.printSet();

            // iterating over the stacks.
            while (!st_stack.isEmpty()) {
                // making a new entry.
                BDD p_st = st_stack.pop();
                int p_j = j_stack.pop().intValue();

                /* Create a new automaton state for our current state 
                  (or use a matching one if it already exists) */
                RawState new_state = new RawState(aut.size(), p_st, p_j);
                int nidx = aut.indexOf(new_state);
                if (nidx == -1) {
                    aut.push(new_state);
                } else {
                    new_state = aut.elementAt(nidx);
                }

                /* Find Y index of current state */
                // find minimal cy and an i
                int p_cy = -1;
                for (int i = 0; i < y_mem[p_j].length; i++) {
                    if (!p_st.and(y_mem[p_j][i]).isZero()) {
                        p_cy = i;
                        break;
                    }
                }
                assert p_cy >= 0 : "Couldn't find p_cy";
                /* Find  X index of current state */
                int p_i = -1;
                for (int i = 0; i < envJustNum; i++) {
			         if (!p_st.and(x_mem[p_j][i][p_cy]).isZero()) {
                        p_i = i;
                        break;
                    }
                }
                assert p_i >= 0 : "Couldn't find p_i";

                // computing the set of env possible successors.
                Vector<BDD> succs = new Vector<BDD>();
                BDD all_succs = env.succ(p_st);
                for (BDDIterator all_states = all_succs.iterator(env
                        .moduleUnprimeVars()); all_states.hasNext();) {
                    BDD sin = (BDD) all_states.next();
                    succs.add(sin);
                }

                // For each env successor, find a strategy successor
                for (Iterator<BDD> iter_succ = succs.iterator(); iter_succ
                        .hasNext();) {
                    BDD primed_cur_succ = Env.prime(iter_succ.next());
                    BDD next_op = Env
                            .unprime(sys.trans().and(p_st).and(primed_cur_succ)
                                    .exist(
                                            env.moduleUnprimeVars().union(
                                                    sys.moduleUnprimeVars())));
                    BDD candidate = Env.FALSE();
                    int jcand = p_j;

                    int local_kind = strategy_kind;
                    while (candidate.isZero() & (local_kind >= 0)) {
                        // a - first successor option in the strategy.
                        // (rho_1 in Piterman; satisfy current goal and move to next goal)
                        if ((local_kind == 3) | (local_kind == 7)
                                | (local_kind == 10) | (local_kind == 13)
                                | (local_kind == 18) | (local_kind == 21)) {
                            int next_p_j = (p_j + 1) % sys.justiceNum();
                            if (!p_st.and(sys.justiceAt(p_j)).isZero()) {
                                BDD opt = next_op.and(z_mem[next_p_j]);
                                if (!opt.isZero()) {
                                    candidate = opt;
                                    jcand = next_p_j;
                                }
                            }
                        }
                        // b - second successor option in the strategy.
                        // (rho_2 in Piterman; move closer to current goal)
                        if ((local_kind == 2) | (local_kind == 5)
                                | (local_kind == 11) | (local_kind == 15)
                                | (local_kind == 17) | (local_kind == 22)) {
                            if (p_cy > 0) {
                                int look_r = 0;
                                // look for the farest r.
                                while ((next_op.and(y_mem[p_j][look_r]).isZero())
                                        & (look_r < p_cy)) {
                                    look_r++;
                                }
                                
                                BDD opt = next_op.and(y_mem[p_j][look_r]);
                                if ((look_r != p_cy) && (!opt.isZero())) {
                                    candidate = opt;
                                }
                            }
                        }

                        // c - third successor option in the strategy.
                        // (rho_3 in Piterman; falsify environment :()
                        if ((local_kind == 1) | (local_kind == 6)
                                | (local_kind == 9) | (local_kind == 14)
                                | (local_kind == 19) | (local_kind == 23)) {
                            if (!p_st.and(env.justiceAt(p_i).not()).isZero()) {
                                BDD opt = next_op.and(x_mem[p_j][p_i][p_cy]);
                                if (!opt.isZero()) {
                                    candidate = opt;
                                }
                            }
                        }

                        // no successor was found yet.
                        assert ((local_kind != 0) & (local_kind != 4)
                                & (local_kind != 8) & (local_kind != 12)
                                & (local_kind != 16) & (local_kind != 20)) : "No successor was found";

                        local_kind--;
                    }

                    // picking one candidate. In JDD satOne is not take
                    // env.unprimeVars().union(sys.unprimeVars()) into its
                    // considerations.
                    // BDD one_cand = candidate.satOne();
                    BDD one_cand = candidate.satOne(env.moduleUnprimeVars().union(
                            sys.moduleUnprimeVars()), false);

                    RawState gsucc = new RawState(aut.size(), one_cand, jcand);
                    idx = aut.indexOf(gsucc); // the equals doesn't consider
                                              // the id number.
                    if (idx == -1) {
                        st_stack.push(one_cand);
                        j_stack.push(jcand);
                        aut.add(gsucc);
                        idx = aut.indexOf(gsucc);
                    }
                    new_state.add_succ(aut.elementAt(idx));
                }
            }
        }

        /* Remove stuttering */
        // TODO: Make this more efficient (and less ugly) if possible
        /*
		int num_removed = 0;
      	   
        for (RawState state1 : aut) {
            int j1 = state1.get_rank();
            for (RawState state2 : state1.get_succ()) {
                int j2 = state2.get_rank();
                if ((j2 == (j1 + 1) % sys.justiceNum()) &&
                     state1.equals(state2, false)) {
                    // Find any states pointing to state1
                    for (RawState state3 : aut) {
                        if (state3.get_succ().indexOf(state1) != -1) {
                            // Redirect transitions to state2
                            state3.del_succ(state1); 
                            state3.add_succ(state2); 
                            // Mark the extra state for deletion 
                            state1.set_rank(-1);
                        }
                    }
                }
            }

            if (state1.get_rank() == -1) {
                num_removed++;
            }
            
        }

        
        System.out.println("Removed " + num_removed + " stutter states.");
        
        */

        /* Print output */

		String res = "";
		for (RawState state : aut) {
            if (state.get_rank() != -1) {
                res += state + "\n";
            }
		}

		System.out.print("\n\n");
		System.out.print(res);
		// return null; // res;
	}
	
	public void generate_safety_aut(BDD ini) {
		Stack<BDD> st_stack = new Stack<BDD>();
		Stack<RawState> aut = new Stack<RawState>();

        BDDIterator ini_iterator = ini.iterator(env.moduleUnprimeVars().union(sys.moduleUnprimeVars()));

        while (ini_iterator.hasNext()) {

            BDD this_ini = (BDD) ini_iterator.next();

            RawState test_st = new RawState(aut.size(), this_ini, 0);

            int idx = -1;
            for (RawState cmp_st : aut) {
                if (cmp_st.equals(test_st, false)) { // search ignoring rank
                    idx = aut.indexOf(cmp_st);
                    break;
                }
            }

            if (idx != -1) {
                // This initial state is already in the automaton
                continue;
            }

            // Otherwise, we need to attach this initial state to the automaton

            st_stack.push(this_ini);

            // iterating over the stacks.
            while (!st_stack.isEmpty()) {
                // making a new entry.
                BDD p_st = st_stack.pop();

                /* Create a new automaton state for our current state 
                  (or use a matching one if it already exists) */
                RawState new_state = new RawState(aut.size(), p_st, 0);
                int nidx = aut.indexOf(new_state);
                if (nidx == -1) {
                    aut.push(new_state);
                } else {
                    new_state = aut.elementAt(nidx);
                }

                BDD next_op = Env.unprime(sys.trans().and(p_st).exist(env.moduleUnprimeVars().union(sys.moduleUnprimeVars())));

                BDDIterator next_iterator = next_op.iterator(env.moduleUnprimeVars().union(sys.moduleUnprimeVars()));
                while (next_iterator.hasNext()) {

                    BDD this_next = (BDD) next_iterator.next();
                    //this_next.printSet();
                    RawState gsucc = new RawState(aut.size(), this_next, 0);
                    idx = aut.indexOf(gsucc); // the equals doesn't consider
                                              // the id number.
                    if (idx == -1) {
                        st_stack.push(this_next);
                        aut.add(gsucc);
                        idx = aut.indexOf(gsucc);
                    }
                    new_state.add_succ(aut.elementAt(idx));
                }

                //System.out.print("------------\n");
            }
        }

        /* Print output */

		String res = "";
		for (RawState state : aut) {
            if (state.get_rank() != -1) {
                res += state + "\n";
            }
		}

		System.out.print("\n\n");
		System.out.print(res);
		// return null; // res;
	}

	public void calculate_counterstrategy(BDD ini) {		
		Stack<BDD> st_stack = new Stack<BDD>();
		Stack<Integer> i_stack = new Stack<Integer>();
		Stack<Integer> j_stack = new Stack<Integer>();	
		Stack<RawCState> aut = new Stack<RawCState>();
		
        BDDIterator ini_iterator = ini.iterator(env.moduleUnprimeVars().union(sys.moduleUnprimeVars()));
        int a = 0;
        BDD this_ini = (BDD) ini_iterator.next();
            
        int idx = -1;
        st_stack.push(this_ini);
        i_stack.push(new Integer(0)); // TODO: is there a better default j?
        j_stack.push(new Integer(-1)); // TODO: is there a better default j?
        this_ini.printSet();

            // iterating over the stacks.
        while (!st_stack.isEmpty()) {
                // making a new entry.
                BDD p_st = st_stack.pop();
                int rank_i = i_stack.pop().intValue();
                int rank_j = j_stack.pop().intValue();
                

                /* Create a new automaton state for our current state 
                  (or use a matching one if it already exists) */
                RawCState new_state = new RawCState(aut.size(), p_st, rank_j, rank_i, Env.FALSE());
                int nidx = aut.indexOf(new_state);
                if (nidx == -1) {
                    aut.push(new_state);
                } else {
                    new_state = aut.elementAt(nidx);
                }
                
                
             
                
                BDD input = null;
                int new_i = -1, new_j = -1;
                // For each env successor
                if (input == null) {                 
                	
                	
                	// computing the set of env possible successors.
	                Vector<BDD> succs = new Vector<BDD>();
	                BDD all_succs = env.succ(p_st);
	                for (BDDIterator all_states = all_succs.iterator(env
	                        .moduleUnprimeVars()); all_states.hasNext();) {
	                    BDD sin = (BDD) all_states.next();
	                    succs.add(sin);
	                }
	                for (Iterator<BDD> iter_succ = succs.iterator(); iter_succ
                    .hasNext();) {
	                	BDD primed_cur_succ = Env.prime(iter_succ.next());
	                	if (!(p_st.and(primed_cur_succ).and(sys.yieldStates(env,Env.FALSE())).isZero())) {
		                	input = primed_cur_succ;
		                	new_i = rank_i;
		                	new_j = rank_j;	
		                	break;		                    	
	                	} 
                	}
	                
	             // computing the set of env possible successors.
	                succs = new Vector<BDD>();
	                all_succs = env.succ(p_st);
	                for (BDDIterator all_states = all_succs.iterator(env
	                        .moduleUnprimeVars()); all_states.hasNext();) {
	                    BDD sin = (BDD) all_states.next();
	                    succs.add(sin);
	                }
	                for (Iterator<BDD> iter_succ = succs.iterator(); iter_succ
                    .hasNext();) {
	                	BDD primed_cur_succ = Env.prime(iter_succ.next());
	                	if (input == null && !(Env.unprime(primed_cur_succ).and(sys.yieldStates(env,Env.FALSE())).isZero())) {
		                	input = primed_cur_succ;
		                	new_i = rank_i;
		                	new_j = rank_j;	
		                	break;		                    	
	                	} 
                	}

	                
	                
		                /* Find Z index of current state */
		                // find minimal Z_a
		                int p_a = -1;
		             // computing the set of env possible successors.
		                succs = new Vector<BDD>();
		                all_succs = env.succ(p_st);
		                for (BDDIterator all_states = all_succs.iterator(env
		                        .moduleUnprimeVars()); all_states.hasNext();) {
		                    BDD sin = (BDD) all_states.next();
		                    succs.add(sin);
		                }
		                for (Iterator<BDD> iter_succ = succs.iterator(); iter_succ
                        .hasNext();) {
		                	BDD primed_cur_succ = Env.prime(iter_succ.next());
		                	
		//                	System.out.println("one succ" + primed_cur_succ);               	
		                    
		                	//BDD primed_cur_succ = iter_succ.next();
		                	
		                	if (!p_st.and(z2_mem[0])
		                    		.and(primed_cur_succ.and((sys.yieldStates(env,(Env.FALSE()))))).isZero()) {
		                        p_a = 0;
		                    }
                    
			                for (int az = 1; az < z2_mem.length; az++) {
			                	if (!p_st.and(z2_mem[az])
			                    		.and(primed_cur_succ.and((sys.yieldStates(env,((z2_mem[az-1])))))).isZero()) {
			                        p_a = az;
			                        az = z2_mem.length;
			                    }
			                }
			                //assert p_a >= 1 : "Couldn't find p_a >= 2";
			                if (input == null && p_a >=0) {
			                	System.out.println("found p_a " + p_a + primed_cur_succ);
			                	input = primed_cur_succ;
			                	new_i = rank_i;
			                	new_j = -1;		
			                	break;
			                }
			                
		                }
		                
		                /* Find  Y index of current state */
		                
		             // computing the set of env possible successors.
		                succs = new Vector<BDD>();
		                all_succs = env.succ(p_st);
		                for (BDDIterator all_states = all_succs.iterator(env
		                        .moduleUnprimeVars()); all_states.hasNext();) {
		                    BDD sin = (BDD) all_states.next();
		                    succs.add(sin);
		                }
		                
		                for (Iterator<BDD> iter_succ = succs.iterator(); iter_succ
                        .hasNext();) {
		                	BDD primed_cur_succ = Env.prime(iter_succ.next()); 
		//                	System.out.println("two succ" + primed_cur_succ); 
                    
			                int p_j = -1;
			                for (int az = 0; az < z2_mem.length; az++) {
			                	for (int j = 0; j < sysJustNum; j++) {  
			                		if (az == 0){
		/*	                			System.out.println("p_st" + p_st); 
			                			System.out.println("p_st.and(z2_mem[az])" + p_st.and(z2_mem[az])); 
			                			System.out.println("primed_cur_succ" + primed_cur_succ); 
			                			System.out.println("sys.yieldStates(env,(y2_mem[j][az]))" + sys.yieldStates(env,(y2_mem[j][az])));			                			
			                			System.out.println("primed_cur_succ.and(sys.yieldStates(env,(y2_mem[j][az])))" + primed_cur_succ.and(sys.yieldStates(env,(y2_mem[j][az])))); 
			                			System.out.println("p_st.and(z2_mem[az]).and(primed_cur_succ.and(sys.yieldStates(env,(y2_mem[j][az]))))" + p_st.and(z2_mem[az]).and(primed_cur_succ.and(sys.yieldStates(env,(y2_mem[j][az]))))); 
			                			System.out.println("sys.yieldStates(env,Env.FALSE())).not()" + sys.yieldStates(env,Env.FALSE()).not());
		*/		                    	if (!(p_st.and(z2_mem[az])
				                    			.and(primed_cur_succ.and(sys.yieldStates(env,(y2_mem[j][az]))))
				                    			//.and(primed_cur_succ.and((env.yieldStates(sys,(y2_mem[j][az]).not())).not()))
				                    			.and(sys.yieldStates(env,Env.FALSE()).not()).isZero())) {
				                    			//.and((env.yieldStates(sys,Env.TRUE()))).isZero())) {
				                    		p_j = j;
				                    		az = z2_mem.length;
				                    		j = sysJustNum;
					                        
					                    }
				                    } else {
				                    	if (!(p_st.and(z2_mem[az]).and(z2_mem[az-1].not())
				                    			.and(primed_cur_succ.and(sys.yieldStates(env,(y2_mem[j][az]))))
				                    			//.and(primed_cur_succ.and((env.yieldStates(sys,(y2_mem[j][az]).not())).not()))
				                    			.and((sys.yieldStates(env,(z2_mem[az-1]))).not()).isZero())) {
				                    			//.and((env.yieldStates(sys,(z2_mem[az-1]).not())))).isZero()) {
			                    			
					                        p_j = j;
					                        az = z2_mem.length;
					                        j = sysJustNum;
					                        
					                    }
				                    }
				                        
			                	}
			                }
			                //assert p_j >= 0 : "Couldn't find p_j >= 1";
		/*	                System.out.println("p_j = "+ p_j);
			                System.out.println("input = "+ input);
			                System.out.println("rank_j = "+ rank_j);
		*/                	
			                if ((input == null || input.equals(primed_cur_succ)) && p_j >= 0 && rank_j == -1) {
		//	                	System.out.println("found p_j");
			                	input = primed_cur_succ;
			                	new_i = rank_i;
			                	new_j = p_j;
			                	break;
			                }
			                
		                }
			                
		             // computing the set of env possible successors.
		                succs = new Vector<BDD>();
		                all_succs = env.succ(p_st);
		                for (BDDIterator all_states = all_succs.iterator(env
		                        .moduleUnprimeVars()); all_states.hasNext();) {
		                    BDD sin = (BDD) all_states.next();
		                    succs.add(sin);
		                }
		                
		                for (Iterator<BDD> iter_succ = succs.iterator(); iter_succ
                        .hasNext();) {
		                	BDD primed_cur_succ = Env.prime(iter_succ.next());  
		  //              	System.out.println("three succ" + primed_cur_succ); 
		                	int p_i = -1;
		                	for (int az = 0; az < z2_mem.length; az++) {                        
		                	  if (az == 0){
			                    	if (rank_j != -1 && !(p_st.and(z2_mem[az])
			                    			//.and(primed_cur_succ.and((env.yieldStates(sys,y2_mem[rank_j][az].not())).not())
			                    			.and(primed_cur_succ.and(sys.yieldStates(env,y2_mem[rank_j][az])))
			                    			//.and(env.yieldStates(sys,Env.TRUE()))).isZero()) {
			                    			.and((sys.yieldStates(env,Env.FALSE())).not())).isZero()) {
			                    		p_i = az;
				                        break;
				                    }
			                    } else {

			                    	//assert false : z2_mem[az-1].not();
		                	  		//BDD z_not1 = z2_mem[az-1];
			                    	//BDD z_not = env.yieldStates(sys,z_not1);
			                    	if (rank_j != -1 && 
			                    			!(p_st.and(z2_mem[az]).and(z2_mem[az-1].not())
			                    			//.and(primed_cur_succ.and((env.yieldStates(sys,(y2_mem[rank_j][az]).not())).not()))
			                    			.and(primed_cur_succ.and((sys.yieldStates(env,y2_mem[rank_j][az]))))
			                    			//.and((env.yieldStates(sys,z2_mem[az-1])).not()).isZero())) {
			                    			.and((sys.yieldStates(env,z2_mem[az-1])).not())).isZero()) {
			                    		p_i = az;
			                    		az = z2_mem.length;
				                    }
			                    }
		                	  if (input == null && p_i >= 0) {
			                	  if (rank_j != -1 && !p_st.and(env.justiceAt(rank_i)).isZero()) {
		//	                		  System.out.println("found p_i");
			                		  input = primed_cur_succ;
			                		  new_i = (rank_i + 1) % env.justiceNum();
			                		  new_j = rank_j;
			                		  break;
			                	  }
			                	  
		                	  }
		                	}
		                }
		                //assert p_i >= 0 : "Couldn't find p_i";
		                
		             // computing the set of env possible successors.
		                succs = new Vector<BDD>();
		                all_succs = env.succ(p_st);
		                for (BDDIterator all_states = all_succs.iterator(env
		                        .moduleUnprimeVars()); all_states.hasNext();) {
		                    BDD sin = (BDD) all_states.next();
		                    succs.add(sin);
		                }
		                for (Iterator<BDD> iter_succ = succs.iterator(); iter_succ
                        .hasNext();) {
		                	BDD primed_cur_succ = Env.prime(iter_succ.next()); 
		     //           	System.out.println("four succ" + primed_cur_succ); 
                			int p_x = -1;
			                for (int az = 0; az < z2_mem.length; az++) {
			                	if (rank_i != -1 && rank_j != -1) {
				                	for (int c = 1; c < x2_mem[rank_j][rank_i][az].length; c++)
				                	{
				                	  if (az == 0){
					                    	if (!(p_st.and(z2_mem[az])
					                    			.and(x2_mem[rank_j][rank_i][az][c])
					                    			.and(x2_mem[rank_j][rank_i][az][c-1].not())
					                    			//.and(primed_cur_succ.and((env.yieldStates(sys,(x2_mem[az][rank_j][rank_i][c-1]).not())).not()))
					                    			.and(primed_cur_succ.and((sys.yieldStates(env,x2_mem[rank_j][rank_i][az][c-1])))
					                    					//.and((env.yieldStates(sys,Env.TRUE())))).isZero())) {
					                    					.and(sys.yieldStates(env,Env.FALSE()).not()))).isZero()) {
					                    		
					                    		p_x = az;
					                    		az = z2_mem.length;
						                        break;
						                    }
					                    } else {
					                    	if (!(p_st.and(z2_mem[az])
					                    			.and(x2_mem[rank_j][rank_i][az][c])
					                    			.and(x2_mem[rank_j][rank_i][az][c-1].not())
					                    			.and(primed_cur_succ.and((sys.yieldStates(env,x2_mem[rank_j][rank_i][az][c-1])))
					                    			//.and(primed_cur_succ.and((env.yieldStates(sys,(x2_mem[rank_j][rank_i][az][c-1]).not())).not()))
				                    					.and((sys.yieldStates(env,z2_mem[az-1])).not()))).isZero()) {
					                    				//.and((env.yieldStates(sys,(z2_mem[az-1]).not())))).isZero())) {
					                    	
						                    	p_x = az;
						                    	az = z2_mem.length;
						                        break;
						                    }
					                    }
				                	}
			                	}
			                }
			                if (input == null && p_x >= 0) {
			                	if (rank_j != -1) {
			//	                	System.out.println("found p_x");
			                		input = primed_cur_succ;
					                		  new_i = rank_i;
					                		  new_j = rank_j;
					                		  break;
			                	}	
			                	
			                }		
		                }	
                }
                
          	System.out.println("input = " + input); 
     		   
		                
                // computing the set of system possible successors.
                Vector<BDD> sys_succs = new Vector<BDD>();               
                BDD all_sys_succs = sys.succ(p_st.and(input));
                if (all_sys_succs.equals(Env.FALSE())) {
                	RawCState gsucc = new RawCState(aut.size(), Env.unprime(input), new_j, new_i, input);
                    idx = aut.indexOf(gsucc); // the equals doesn't consider
                                              // the id number.
                    if (idx == -1) {                       
                        aut.add(gsucc);
                        idx = aut.indexOf(gsucc);
                    }
                    new_state.add_succ(aut.elementAt(idx));
                	continue;                	
                }               	
                
                for (BDDIterator all_sys_states = all_sys_succs.iterator(sys
                        .moduleUnprimeVars()); all_sys_states.hasNext();) {
                    BDD sin = (BDD) all_sys_states.next();
                    sys_succs.add(sin);
                }
                
                

                // For each system successor, find a strategy successor
                for (Iterator<BDD> iter_succ = sys_succs.iterator(); iter_succ
                        .hasNext();) {
                    BDD sys_succ = iter_succ.next().and(Env.unprime(input));
                    /*BDD next_op = Env.unprime(sys.trans().and(p_st).and(primed_cur_succ)
                            .exist(
                                    env.moduleUnprimeVars().union(
                                            sys.moduleUnprimeVars())));
            BDD candidate = Env.FALSE();
            int jcand = p_j;*/

                    
                    RawCState gsucc = new RawCState(aut.size(), sys_succ, new_j, new_i, input);
                    idx = aut.indexOf(gsucc); // the equals doesn't consider
                                              // the id number.
                    if (idx == -1) {
                        st_stack.push(sys_succ);
                        i_stack.push(new_i);
                        j_stack.push(new_j);
                        aut.add(gsucc);
                        idx = aut.indexOf(gsucc);
                    }
                    new_state.add_succ(aut.elementAt(idx));
                }
            }
        //}       


        String res = "";
		for (RawCState state : aut) {
            if (state.get_rank_i() != -1) {
                res += state + "\n";
            }
		}

		System.out.print("\n\n");
		System.out.print(res);
		// return null; // res;
	}
	
	/*public void calculate_countertrace(BDD ini) {		
		Stack<BDD> st_stack = new Stack<BDD>();
		Stack<Integer> i_stack = new Stack<Integer>();
		Stack<Integer> j_stack = new Stack<Integer>();	
		/*Stack<Integer> i_new_stack = new Stack<Integer>();
		Stack<Integer> j_new_stack = new Stack<Integer>();			
		*/
	/*	//Stack<BDD> in_stack = new Stack<BDD>();		
		Stack<RawCState> aut = new Stack<RawCState>();
		// FDSModule res = new FDSModule("strategy");

        BDDIterator ini_iterator = ini.iterator(env.moduleUnprimeVars().union(sys.moduleUnprimeVars()));
        int a = 0;
        //while (ini_iterator.hasNext()) {

            BDD this_ini = (BDD) ini_iterator.next();

        //    RawCState test_st = new RawCState(aut.size(), this_ini, 0, -1, Env.FALSE());

   /*         int idx = -1;
        /*    for (RawCState cmp_st : aut) {
                if (cmp_st.equals(test_st, false)) { // search ignoring rank
                    idx = aut.indexOf(cmp_st);
                    break;
                }
            }

            if (idx != -1) {
                // This initial state is already in the automaton
                continue;
            }
            if (a == 1) {
                break;
            }
            a++;*/
            // Otherwise, we need to attach this initial state to the automaton

       /*     st_stack.push(this_ini);
            i_stack.push(new Integer(0)); // TODO: is there a better default j?
            j_stack.push(new Integer(-1)); // TODO: is there a better default j?
            //i_stack_new.push(new Integer(0)); // TODO: is there a better default j?
            //j_stack_new.push(new Integer(0)); // TODO: is there a better default j?
            //in_stack.push(Env.FALSE());
            this_ini.printSet();

            // iterating over the stacks.
            while (!st_stack.isEmpty()) {
                // making a new entry.
                BDD p_st = st_stack.pop();
                int rank_i = i_stack.pop().intValue();
                int rank_j = j_stack.pop().intValue();
                //int p_i_n = i_stack_new.pop().intValue();
                //int p_j_n = j_stack_new.pop().intValue();
                //BDD p_in = in_stack.pop();
                

                /* Create a new automaton state for our current state 
                  (or use a matching one if it already exists) */
       /*         RawCState new_state = new RawCState(aut.size(), p_st, rank_i, rank_j, Env.FALSE());
                int nidx = aut.indexOf(new_state);
                if (nidx == -1) {
                    aut.push(new_state);
                } else {
                    new_state = aut.elementAt(nidx);
                }
                
                
             // computing the set of env possible successors.
                Vector<BDD> succs = new Vector<BDD>();
                BDD all_succs = env.succ(p_st);
                for (BDDIterator all_states = all_succs.iterator(env
                        .moduleUnprimeVars()); all_states.hasNext();) {
                    BDD sin = (BDD) all_states.next();
                    succs.add(sin);
                }
                
                BDD input = Env.FALSE();
                int new_i = -1, new_j = -1;
                // For each env successor
                //while (input.isZero()) {                
		                /* Find Z index of current state */
		                // find minimal Z_a
		/*                for (Iterator<BDD> iter_succ = succs.iterator(); iter_succ
                        .hasNext();) {
		                	int p_a = -1;
			                
		                	//BDD primed_cur_succ = Env.prime(iter_succ.next());
		                	BDD primed_cur_succ = iter_succ.next();
                    
			                for (int az = 1; az < z2_mem.length; az++) {
			                    if (!p_st.and(z2_mem[az])
			                    		.and((env.yieldStates(sys,(primed_cur_succ.and(z2_mem[az-1])).not())).not()).isZero()) {
			                        p_a = az;
			                        break;
			                    }
			                }
			                //assert p_a >= 1 : "Couldn't find p_a >= 2";
			                if (p_a >=1) {
			                	input = input.or(primed_cur_succ);
			                	new_i = rank_i;
			                	new_j = -1;		                	
			                }
		                }
		                
		                /* Find  Y index of current state */
		                
		                
		 /*               for (Iterator<BDD> iter_succ = succs.iterator(); iter_succ
                        .hasNext();) {
		                	BDD primed_cur_succ = Env.prime(iter_succ.next());  
                    
			                int p_j = -1;
			                for (int az = 0; az < z2_mem.length; az++) {
			                	for (int j = 0; j < y2_mem[az].length; j++) {                        
				                    if (az == 0){
				                    	if (!p_st.and(z2_mem[az])
				                    			//.and(primed_cur_succ.and((env.yieldStates(sys,(y2_mem[az][j]).not())).not()))
				                    			.and(primed_cur_succ.and(env.controlStates(sys,(y2_mem[az][j]))))
				                    			//.and(env.yieldStates(sys,Env.TRUE())).isZero()) {
				                    			.and((env.controlStates(sys,Env.FALSE())).not()).isZero()) {
				                    		p_j = j;
					                        break;
					                    }
				                    } else {
				                    	if (!p_st.and(z2_mem[az]).and(z2_mem[az-1].not())
				                    			//.and(primed_cur_succ.and((env.yieldStates(sys,(y2_mem[az][j]).not())).not()))
				                    			.and(primed_cur_succ.and(env.controlStates(sys,(y2_mem[az][j]))))
				                    			//.and((env.yieldStates(sys,(z2_mem[az-1]).not()))).isZero()) {
				                    			.and((env.controlStates(sys,(z2_mem[az-1]).not())).not()).isZero()) {
					                        p_j = j;
					                        break;
					                    }
				                    }
				                        
			                	}
			                }
			                //assert p_j >= 0 : "Couldn't find p_j >= 1";
			                if (p_j >= 0) {
			                	input = input.or(primed_cur_succ);
			                	new_i = rank_i;
			                	new_j = p_j;
			                }
		                }
			                
		                
		                for (Iterator<BDD> iter_succ = succs.iterator(); iter_succ
                        .hasNext();) {
		                	BDD primed_cur_succ = Env.prime(iter_succ.next());  
		                	int p_i = -1;
		                	for (int az = 0; az < z2_mem.length; az++) {                        
		                	  if (az == 0){
			                    	if (rank_j != -1 && !p_st.and(z2_mem[az])
			                    			//.and(primed_cur_succ.and(env.yieldStates(sys,y2_mem[az][rank_j].not())).not())
			                    			.and(primed_cur_succ.and(env.controlStates(sys,y2_mem[az][rank_j])))
			                    			//.and(env.yieldStates(sys,Env.TRUE())).isZero()) {
			                    			.and((env.controlStates(sys,Env.FALSE())).not()).isZero()) {
			                    		p_i = az;
				                        break;
				                    }
			                    } else {

			                    	//assert false : z2_mem[az-1].not();
		                	  		//BDD z_not1 = z2_mem[az-1];
			                    	//BDD z_not = env.yieldStates(sys,z_not1);
			                    	if (rank_j != -1 && 
			                    			!p_st.and(z2_mem[az]).and(z2_mem[az-1].not())
			                    			.and(primed_cur_succ.and((env.controlStates(sys,y2_mem[az][rank_j]))))
			                    			//.and(primed_cur_succ.and((env.yieldStates(sys,y2_mem[az][rank_j].not())).not()))
			                    			.and(env.yieldStates(sys,z2_mem[az-1].not())).isZero()) {
			                    			//.and((env.controlStates(sys,z2_mem[az-1])).not()).isZero()) {
			                    		p_i = az;
				                        break;
				                    }
			                    }
		                	  if (p_i >= 0) {
			                	  if (rank_j != -1 && !p_st.and(env.justiceAt(rank_i)).isZero()) {
			                		  input = input.or(primed_cur_succ);
			                		  new_i = (rank_i + 1) % env.justiceNum();
			                		  new_j = rank_j;
			                	  }
		                	  }
		                	}
		                }
		                //assert p_i >= 0 : "Couldn't find p_i";
		                
		                for (Iterator<BDD> iter_succ = succs.iterator(); iter_succ
                        .hasNext();) {
		                	BDD primed_cur_succ = Env.prime(iter_succ.next());  
                			int p_x = -1;
			                for (int az = 0; az < z2_mem.length; az++) {
			                	if (rank_i != -1 && rank_j != -1) {
				                	for (int c = 1; c < x2_mem[az][rank_j][rank_i].length; c++)
				                	{
				                	  if (az == 0){
					                    	if (!p_st.and(z2_mem[az])
					                    			.and(x2_mem[az][rank_j][rank_i][c])
					                    			.and(x2_mem[az][rank_j][rank_i][c-1].not())
					                    			.and(primed_cur_succ.and(env.controlStates(sys,x2_mem[az][rank_j][rank_i][c-1]))
					                    					.and((env.controlStates(sys,Env.FALSE())).not())).isZero()) {
					                    		p_x = az;
						                        break;
						                    }
					                    } else {
					                    	if (!p_st.and(z2_mem[az])
					                    			.and(x2_mem[az][rank_j][rank_i][c])
					                    			.and(x2_mem[az][rank_j][rank_i][c-1].not())
					                    			.and(primed_cur_succ.and(env.controlStates(sys,x2_mem[az][rank_j][rank_i][c-1]))
					                    					.and((env.controlStates(sys,z2_mem[az-1])).not())).isZero()) {
						                    	p_x = az;
						                        break;
						                    }
					                    }
				                	}
			                	}
			                }
			                if (p_x >= 0) {
			                	if (rank_j != -1) {
			                		input = input.or(primed_cur_succ);
					                		  new_i = rank_i;
					                		  new_j = rank_j;
			                	}			
			                }		
		                //}	
                }	
		        if (!input.equals(all_succs)) System.out.println("Not Unsat!");
		        //else{
		                
	                
	                
	                
	                for (BDDIterator iter_input = input.iterator(env.moduleUnprimeVars()); iter_input.hasNext();) {
              
	                	BDD in = (BDD) iter_input.next();
	                	// computing the set of system possible successors.
		                Vector<BDD> sys_succs = new Vector<BDD>();
		                BDD all_sys_succs = sys.succ(p_st.and(in));
		                for (BDDIterator all_sys_states = all_sys_succs.iterator(sys
		                        .moduleUnprimeVars()); all_sys_states.hasNext();) {
		                    BDD sin = (BDD) all_sys_states.next();
		                    sys_succs.add(sin);
		                }
		                
		                // For each system successor, find a strategy successor
		                for (Iterator<BDD> iter_succ = sys_succs.iterator(); iter_succ
		                        .hasNext();) {
		                	
		                    BDD sys_succ = iter_succ.next().and(in);
		                    /*BDD next_op = Env.unprime(sys.trans().and(p_st).and(primed_cur_succ)
		                            .exist(
		                                    env.moduleUnprimeVars().union(
		                                            sys.moduleUnprimeVars())));
		            BDD candidate = Env.FALSE();
		            int jcand = p_j;*/
		/*
		                    
		                    RawCState gsucc = new RawCState(aut.size(), sys_succ, new_i, new_j, in);
		                    idx = aut.indexOf(gsucc); // the equals doesn't consider
		                                              // the id number.
		                    if (idx == -1) {
		                        st_stack.push(sys_succ);
		                        i_stack.push(new_i);
		                        j_stack.push(new_j);
		                        aut.add(gsucc);
		                        idx = aut.indexOf(gsucc);
		                    }
		                    new_state.add_succ(aut.elementAt(idx));
		                }
	                }
	            //}
            }
        //}       


        String res = "";
		for (RawCState state : aut) {
            if (state.get_rank_i() != -1) {
                res += state + "\n";
            }
		}

		System.out.print("\n\n");
		System.out.print(res);
		// return null; // res;
	}*/
	

		
	@SuppressWarnings("unused")
	private class RawState {
		private int id;
		private int rank;
		private BDD state;
		private Vector<RawState> succ;

		public RawState(int id, BDD state, int rank) {
			this.id = id;
			this.state = state;
			this.rank = rank;
			succ = new Vector<RawState>(10);
		}

		public void add_succ(RawState to_add) {
			succ.add(to_add);
		}

		public void del_succ(RawState to_del) {
			succ.remove(to_del);
		}

		public BDD get_state() {
			return this.state;
		}

		public int get_rank() {
			return this.rank;
		}

		public void set_rank(int rank) {
			this.rank = rank;
		}

		public Vector<RawState> get_succ() {
			//RawState[] res = new RawState[this.succ.size()];
			//this.succ.toArray(res);
			return this.succ;
		}

		public boolean equals(Object other) {
            return this.equals(other, true);
        }

		public boolean equals(Object other, boolean use_rank) {
			if (!(other instanceof RawState))
				return false;
			RawState other_raw = (RawState) other;
			if (other_raw == null)
				return false;

            if (use_rank) {
                return ((this.rank == other_raw.rank) & (this.state
                        .equals(other_raw.state)));
            } else {
                return (this.state.equals(other_raw.state));
            }
		}

		public String toString() {
			String res = "State " + id + " with rank " + rank + " -> "
					+ state.toStringWithDomains(Env.stringer) + "\n";
			if (succ.isEmpty()) {
				res += "\tWith no successors.";
			} else {
				RawState[] all_succ = new RawState[succ.size()];
				succ.toArray(all_succ);
				res += "\tWith successors : " + all_succ[0].id;
				for (int i = 1; i < all_succ.length; i++) {
					res += ", " + all_succ[i].id;
				}
			}
			return res;
		}
	}
	
	
	private class RawCState {
		private int id;
		private int rank_i;//_old, rank_j_old;
		private int rank_j;//i_new, rank_j_new;
		private BDD input;
		private BDD state;
		private Vector<RawCState> succ;
		
		/*public RawCState(int id, BDD state, int rank_i_old, int rank_i_new, int rank_j_old, int rank_j_new, BDD input) {
			this.id = id;
			this.state = state;
			this.rank_i_old = rank_i_old;
			this.rank_j_old = rank_j_old;
			this.rank_i_new = rank_i_new;
			this.rank_j_new = rank_j_new;
			this.input = input;
		}*/
		
		public RawCState(int id, BDD state, int rank_j, int rank_i, BDD input) {
			this.id = id;
			this.state = state;
			this.rank_i = rank_i;
			this.rank_j = rank_j;
			this.input = input;
			succ = new Vector<RawCState>(10);
		}

		public void add_succ(RawCState to_add) {
			succ.add(to_add);
		}

		public void del_succ(RawCState to_del) {
			succ.remove(to_del);
		}


		public BDD get_input() {
			return this.input;
		}

		public void set_input(BDD input) {
			this.input = input;
		}

		public BDD get_state() {
			return this.state;
		}

		public int get_rank_i() {
			return this.rank_i;
		}

		public void set_rank_i(int rank) {
			this.rank_i = rank;
		}
		
		public int get_rank_j() {
			return this.rank_j;
		}

		public void set_rank_j(int rank) {
			this.rank_j = rank;
		}
		
		/*public int get_rank_i_new() {
			return this.rank_i_new;
		}

		public void set_rank_i_new(int rank) {
			this.rank_i_new = rank;
		}
		
		public int get_rank_j_new() {
			return this.rank_j_new;
		}

		public void set_rank_j_new(int rank) {
			this.rank_j_new = rank;
		}*/
		

		public boolean equals(Object other) {
            return this.equals(other, true);
        }

		public boolean equals(Object other, boolean use_rank) {
			if (!(other instanceof RawCState))
				return false;
			RawCState other_raw = (RawCState) other;
			if (other_raw == null)
				return false;
            if (use_rank) {
                //return ((this.rank_i_old == other_raw.rank_i_old) & (this.rank_j_old == other_raw.rank_j_old) & (this.rank_i_new == other_raw.rank_i_new) & (this.rank_j_new == other_raw.rank_j_new) & 
            	return ((this.rank_i == other_raw.rank_i) & (this.rank_j == other_raw.rank_j) &
                		(this.state.equals(other_raw.state)));
            } else {
                return ((this.state.equals(other_raw.state)));
            }
		}

		
		/*public String toString() {
			String res = "State " + id + " with rank_i " + rank_i + " with rank_j " + rank_j + " -> "
					+ state.toStringWithDomains(Env.stringer) + "\n";
			if (succ.isEmpty()) {
				res += "\tWith no successors.";
			} else {
				RawCState[] all_succ = new RawCState[succ.size()];
				succ.toArray(all_succ);
				res += "\tWith successors : " + all_succ[0].id;
				for (int i = 1; i < all_succ.length; i++) {
					res += ", " + all_succ[i].id;
				}
			}
			res += "\tWith input : " + input.toStringWithDomains(Env.stringer) + "\n";			
			return res;
		}*/
		
		public String toString() {
			String res = "State " + id + " with rank (" + rank_i + "," + rank_j + ") -> " 
				+ state.toStringWithDomains(Env.stringer) + "\n";
			if (succ.isEmpty()) {
				res += "\tWith no successors.";				
			} else {
				RawCState[] all_succ = new RawCState[succ.size()];
				succ.toArray(all_succ);
				res += "\tWith successors : " + all_succ[0].id;
				for (int i = 1; i < all_succ.length; i++) {
					res += ", " + all_succ[i].id;
				}
			}
			return res;
		}
	}

	/**
	 * <p>
	 * Getter for the environment player.
	 * </p>
	 * 
	 * @return The environment player.
	 */
	public ModuleWithWeakFairness getEnvPlayer() {
		return env;
	}

	/**
	 * <p>
	 * Getter for the system player.
	 * </p>
	 * 
	 * @return The system player.
	 */
	public ModuleWithWeakFairness getSysPlayer() {
		return sys;
	}

	/**
	 * <p>
	 * Getter for the environment's winning states.
	 * </p>
	 * 
	 * @return The environment's winning states.
	 */
	public BDD sysWinningStates() {
		return player2_winning;
	}

	/**
	 * <p>
	 * Getter for the system's winning states.
	 * </p>
	 * 
	 * @return The system's winning states.
	 */
	public BDD envWinningStates() {
		return player2_winning.not();
	}

	public BDD gameInitials() {
		return getSysPlayer().initial().and(getEnvPlayer().initial());
	}

	public BDD[] playersWinningStates() {
		return new BDD[] { envWinningStates(), sysWinningStates() };
	}

	public BDD firstPlayersWinningStates() {
		return envWinningStates();
	}

	public BDD secondPlayersWinningStates() {
		return sysWinningStates();
	}

}
