import java.util.Iterator;
import java.util.Stack;
import java.util.Vector;

import net.sf.javabdd.BDD;
import net.sf.javabdd.BDD.BDDIterator;
import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.ModuleWithWeakFairness;
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
 * @author yaniv sa'ar.
 * 
 */
public class GROneGame {
	private ModuleWithWeakFairness env;
	private ModuleWithWeakFairness sys;
	private BDD player2_winning;

	// p2_winning in GRGAmes are !p1_winning

	public GROneGame(ModuleWithWeakFairness env, ModuleWithWeakFairness sys)
			throws GameException {
		if ((env == null) || (sys == null)) {
			throw new GameException(
					"cannot instanciate a GR[1] Game with an empty player.");
		}

		this.env = env;
		this.sys = sys;

		// for now I'm giving max_y 50, at the end I'll cut it (and during, I'll
		// extend if needed. (using vectors only makes things more complicate
		// since we cannot instantiate vectors with new vectors)
		x_mem = new BDD[sys.justiceNum()][env.justiceNum()][50];
		y_mem = new BDD[sys.justiceNum()][50];
		z_mem = new BDD[sys.justiceNum()];
		this.player2_winning = this.calculate_win();
	}

	public BDD[][][] x_mem;
	public BDD[][] y_mem;
	public BDD[] z_mem;

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
			for (int j = 0; j < sys.justiceNum(); j++) {
				cy = 0;
				y = Env.FALSE();
				for (iterY = new FixPoint<BDD>(); iterY.advance(y);) {
					BDD start = sys.justiceAt(j).and(env.yieldStates(sys, z))
							.or(env.yieldStates(sys, y));
					y = Env.FALSE();
					for (int i = 0; i < env.justiceNum(); i++) {
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

		st_stack.push(ini);
		j_stack.push(new Integer(0));

		// iterating over the stacks.
		while (!st_stack.isEmpty()) {
			// making a new entry.
			BDD p_st = st_stack.pop();
			int p_j = j_stack.pop().intValue();
			RawState new_state = new RawState(aut.size(), p_st, p_j);
			int nidx = aut.indexOf(new_state);
			if (nidx == -1) {
				aut.push(new_state);
			} else {
				new_state = aut.elementAt(nidx);
			}

			// find minimal cy and an i
			int p_cy = -1;
			for (int i = 0; i < y_mem[p_j].length; i++) {
				if (!p_st.and(y_mem[p_j][i]).isZero()) {
					p_cy = i;
					break;
				}
			}
			assert p_cy >= 0 : "Couldn't find p_cy";
			int p_i = -1;
			for (int i = 0; i < env.justiceNum(); i++) {
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
				int idx = aut.indexOf(gsucc); // the equals doesn't consider
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

		String res = "";
		RawState[] all_st = new RawState[aut.size()];
		aut.toArray(all_st);
		for (int i = 0; i < all_st.length; i++) {
			res += all_st[i] + "\n";
		}

		System.out.print("\n\n");
		System.out.print(res);
		// return null; // res;
	}

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

		public BDD get_state() {
			return this.state;
		}

		public int get_rank() {
			return this.rank;
		}

		public BDD[] get_succ() {
			BDD[] res = new BDD[this.succ.size()];
			this.succ.toArray(res);
			return res;
		}

		public boolean equals(Object other) {
			if (!(other instanceof RawState))
				return false;
			RawState other_raw = (RawState) other;
			if (other_raw == null)
				return false;

			return ((this.rank == other_raw.rank) & (this.state
					.equals(other_raw.state)));
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
