/*******************************************************************************
 * SolvePOMDP
 * Copyright (C) 2017 Erwin Walraven
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *******************************************************************************/

package solver;

import java.util.ArrayList;

import lpsolver.LPModel;

import program.POMDP;
import program.SolverProperties;
import pruning.PruneMethod;
import pruning.PrunePolicyGraph;

/**
 * Solving POMDPs using generalized incremental pruning with or without accelerated pruning
 */

public class SolverExact implements Solver {
	private SolverProperties sp;
	private LPModel lp;
	private PruneMethod pm;
	
	private POMDP pomdp;
	private ArrayList<AlphaVector> V0;
	private long totalSolveTime = 0;
	private double expectedValue;
	
	public SolverExact(SolverProperties solverProperties, LPModel lp, PruneMethod pm) {
		this.sp = solverProperties;
		this.lp = lp;
		this.pm = pm;
	}

	/**
	 * Returns the type of the algorithm used
	 */
	public String getType() {
		return "exact";
	}

	/**
	 * Returns the total running time in seconds
	 */
	public double getTotalSolveTime() {
		return totalSolveTime * 0.001;
	}

	/**
	 * Solve a POMDP optimally and return a set of alphavectors
	 * @param pomdp a POMDP object
	 */
	public ArrayList<AlphaVector> solve(POMDP pomdp) {
		assert pm != null && lp != null : pm+" "+lp;
		this.pomdp = pomdp;
		
		totalSolveTime = 0;
		
		long startTime = System.currentTimeMillis();
		
		// create V_0
		V0 = new ArrayList<AlphaVector>();
		for(int a=0; a<pomdp.getNumActions(); a++) {
			double[] vectorEntries = new double[pomdp.getNumStates()];
			
			for(int s=0; s<pomdp.getNumStates(); s++) {
				vectorEntries[s] = pomdp.getReward(s, a);
			}
			
			AlphaVector av = new AlphaVector(vectorEntries);
			V0.add(av);
			av.setAction(a);
		}
		
		// execute dynamic programming stages
		ArrayList<AlphaVector> V = V0;
		double bellmanDifference = Double.POSITIVE_INFINITY;
		int stage = 1;
		
		OutputFileWriter.dumpValueFunction(pomdp, V, sp.getOutputDir()+"/"+pomdp.getInstanceName()+".alpha"+stage, sp.dumpActionLabels());
		
		System.out.println();
		System.out.println("=== RUN POMDP SOLVER ===");
		System.out.println("Algorithm: "+pm.getName());
		System.out.println();
		System.out.println("Stage 1: "+V0.size()+" vectors");
		
		while(true) {
			// execute new DP stage
			stage++;
			ArrayList<AlphaVector> Vnext = getNextV(V);
			
			if(sp.getFixedStages() == -1) {
				bellmanDifference = Math.min(bellmanDifference, getBellmanDifference(V, Vnext));
			}
			
			V = Vnext;
			
			// print output
			double elapsed = (System.currentTimeMillis() - startTime) * 0.001;
			System.out.println("Stage "+stage+": "+Vnext.size()+" vectors, diff "+bellmanDifference+", time elapsed "+elapsed+" sec");
			
			// dump new value function to file
			OutputFileWriter.dumpValueFunction(pomdp, V, sp.getOutputDir()+"/"+pomdp.getInstanceName()+".alpha"+stage, sp.dumpActionLabels());
			
			// stop if value function has converged or a fixed number of stages is reached
			if((sp.getFixedStages() != -1 && stage == sp.getFixedStages()) || bellmanDifference < sp.getValueFunctionTolerance() || elapsed > sp.getTimeLimit()) {
				break;
			}
		}
		
		// if we need to dump a policy graph, then we execute one more iteration with additional bookkeeping
		if(sp.dumpPolicyGraph()) {
			// temporarily replace pruning method
			PruneMethod tempPruneMethod = this.pm;
			this.pm = new PrunePolicyGraph();
			this.pm.setLPModel(lp);
			
			// compute new value function
			stage++;
			ArrayList<AlphaVector> Vnext = getNextV(V);
			bellmanDifference = Math.min(bellmanDifference, getBellmanDifference(V, Vnext));
			V = Vnext;
			double elapsed = (System.currentTimeMillis() - startTime) * 0.001;
			System.out.println("Stage "+stage+": "+Vnext.size()+" vectors, diff "+bellmanDifference+", time elapsed "+elapsed+" sec");
			OutputFileWriter.dumpValueFunction(pomdp, V, sp.getOutputDir()+"/"+pomdp.getInstanceName()+".alpha"+stage, sp.dumpActionLabels());
			
			// restore original pruning method
			this.pm = tempPruneMethod;
		}
		
		totalSolveTime = (System.currentTimeMillis() - startTime);
		expectedValue = AlphaVector.getValue(pomdp.getInitialBelief().getBelief(), V);
		
		String outputFileAlpha = sp.getOutputDir()+"/"+pomdp.getInstanceName()+".alpha";
		OutputFileWriter.dumpValueFunction(pomdp, V, outputFileAlpha, sp.dumpActionLabels());
		
		String outputFilePG = "";
		if(sp.dumpPolicyGraph()) {
			outputFilePG = sp.getOutputDir()+"/"+pomdp.getInstanceName()+".pg";
			OutputFileWriter.dumpPolicyGraph(pomdp, V, outputFilePG, sp.dumpActionLabels());
		}
		
		return V;
	}
	
	/**
	 * Compute a new value function using a dynamic programming stage
	 * @param V value function stage i
	 * @return value function stage i+1
	 */
	private ArrayList<AlphaVector> getNextV(ArrayList<AlphaVector> V) {
		int nVectors = V.size();
		
		// generate g vectors
		AlphaVector[][][] g = new AlphaVector[nVectors][pomdp.getNumActions()][pomdp.getNumObservations()];
		
		for(int k=0; k<nVectors; k++) {
			for(int a=0; a<pomdp.getNumActions(); a++) {
				for(int o=0; o<pomdp.getNumObservations(); o++) {
					double[] vectorEntries = new double[pomdp.getNumStates()];
					
					for(int s=0; s<pomdp.getNumStates(); s++) {
						double entry = 0.0;
						
						for(int sNext=0; sNext<pomdp.getNumStates(); sNext++) {
							entry += pomdp.getObservationProbability(a, sNext, o) * pomdp.getTransitionProbability(s, a, sNext) * V.get(k).getEntry(sNext);
						}
						
						vectorEntries[s] = entry;
					}
					
					g[k][a][o] = new AlphaVector(vectorEntries);
				}
			}
		}
		
		// generate G_a_o sets
		ArrayList<VectorSetCollection> Ga = new ArrayList<VectorSetCollection>();
		for(int a=0; a<pomdp.getNumActions(); a++) {
			VectorSetCollection vsc = new VectorSetCollection();
			
			for(int o=0; o<pomdp.getNumObservations(); o++) {
				ArrayList<AlphaVector> G = new ArrayList<AlphaVector>();
				
				for(int k=0; k<nVectors; k++) {
					AlphaVector alpha_0 = V0.get(a);
					AlphaVector alpha_k = g[k][a][o];
					AlphaVector av = createBackProjection(alpha_0, alpha_k);
					av.setAction(a);
					av.setIndex(k);
					av.setObs(o);
					G.add(av);
				}
				
				vsc.addVectorSet(G);
			}
			
			assert vsc.size() == pomdp.getNumObservations();
			
			Ga.add(vsc);
		}
		
		assert Ga.size() == pomdp.getNumActions();
		
		// generate G_a sets
		ArrayList<ArrayList<AlphaVector>> G = new ArrayList<ArrayList<AlphaVector>>();
		for(int a=0; a<pomdp.getNumActions(); a++) {
			VectorSetCollection vsc = Ga.get(a);
			G.add(pm.crossSum(vsc));
		}
		
		// merge G_a sets
		ArrayList<AlphaVector> Vnext = pm.mergeSets(G);
		
		return Vnext;
	}
	
	/**
	 * Compute vector k of the G_a^o set using Equation 22
	 * @param alpha_0 immediate reward vector
	 * @param alpha_k vector k to be back-projected
	 * @return back-projected vector
	 */
	private AlphaVector createBackProjection(AlphaVector alpha_0, AlphaVector alpha_k) {
		double[] newEntries = new double[pomdp.getNumStates()];
		
		for(int s=0; s<pomdp.getNumStates(); s++) {
			newEntries[s] = (1.0 / ((double) pomdp.getNumObservations())) * alpha_0.getEntry(s) + pomdp.getDiscountFactor() * alpha_k.getEntry(s);
		}
		
		return new AlphaVector(newEntries);
	}
	
	/**
	 * Compute Bellman difference between two successive value functions
	 * @param oldVectors value function stage i
	 * @param newVectors value function stage i+1
	 * @return difference between both value functions
	 */
	private double getBellmanDifference(ArrayList<AlphaVector> oldVectors, ArrayList<AlphaVector> newVectors) {
		double maxDiff = Double.NEGATIVE_INFINITY;
		
		for(AlphaVector av : newVectors) {
			maxDiff = Math.max(maxDiff, lp.getMaxValueDiff(av, oldVectors));
		}
		
		if(pomdp.getMinReward() < 0.0) {
			for(AlphaVector av : oldVectors) {
				maxDiff = Math.max(maxDiff, lp.getMaxValueDiff(av, newVectors));
			}
		}
		
		return maxDiff;
	}
	
	/**
	 * Get expected value of the solution
	 * @return expected value
	 */
	public double getExpectedValue() {
		return expectedValue;
	}
}
