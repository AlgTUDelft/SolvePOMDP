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
import java.util.HashSet;
import java.util.List;
import java.util.Random;

import program.POMDP;
import program.SolverProperties;

/**
 * Solving POMDPs using point-based value iteration
 */

public class SolverApproximate implements Solver {	
	private Random rnd;
	private SolverProperties sp;
	private long totalSolveTime = 0;
	private double expectedValue;
	
	public SolverApproximate(SolverProperties solverProperties, Random rnd) {
		this.rnd = rnd;
		this.sp = solverProperties;
	}
	
	public String getType() {
		return "approximate";
	}

	public double getTotalSolveTime() {
		return totalSolveTime * 0.001;
	}
	
	private ArrayList<BeliefPoint> getBeliefPoints(POMDP pomdp) {
		ArrayList<BeliefPoint> B = new ArrayList<BeliefPoint>();
		HashSet<BeliefPoint> Bset = new HashSet<BeliefPoint>();
		B.add(pomdp.getInitialBelief());
		Bset.add(pomdp.getInitialBelief());
		
		for(int run=0; run<sp.getBeliefSamplingRuns(); run++) {
			BeliefPoint b = pomdp.getInitialBelief();
			
			for(int step=0; step<sp.getBeliefSamplingSteps(); step++) {
				pomdp.prepareBelief(b);
				
				// select action and observation
				int action = rnd.nextInt(pomdp.getNumActions());
				ProbabilitySample ps = new ProbabilitySample(rnd);
				for(int o=0; o<pomdp.getNumObservations(); o++) {
					double prob = b.getActionObservationProbability(action, o);
					if(prob > 1.0) prob = 1.0;
					ps.addItem(o, prob);
				}
				int observation = ps.sampleItem();
				
				// find new belief point
				BeliefPoint bao = pomdp.updateBelief(b, action, observation);
				bao.setHistory(b.getHistoryCopy());
				bao.addToHistory(action);
				bao.addToHistory(observation);
				
				// add belief point and prepare for next step
				if(!Bset.contains(bao)) {
					B.add(bao);
					Bset.add(bao);
				}
				
				b = bao;
			}
		}
		
		// add corner beliefs
		for(int s=0; s<pomdp.getNumStates(); s++) {
			double[] beliefEntries = new double[pomdp.getNumStates()];
			beliefEntries[s] = 1.0;
			B.add(new BeliefPoint(beliefEntries));
		}
		
		return B;
	}
	
	private ArrayList<AlphaVector> backupStage(POMDP pomdp, ArrayList<AlphaVector> immediateRewards, ArrayList<AlphaVector> V, ArrayList<BeliefPoint> B) {
		int nStates = pomdp.getNumStates();
		int nActions = pomdp.getNumActions();
		int nObservations = pomdp.getNumObservations();
		
		ArrayList<AlphaVector> Vnext = new ArrayList<AlphaVector>();
		List<BeliefPoint> Btilde = new ArrayList<BeliefPoint>();
		Btilde.addAll(B);
		
		// initialize gao vectors
		AlphaVector[][][] gkao = new AlphaVector[V.size()][nActions][nObservations];
		for(int k=0; k<V.size(); k++) {
			for(int a=0; a<nActions; a++) {
				for(int o=0; o<nObservations; o++) {
					double[] entries = new double[nStates];
					
					for(int s=0; s<nStates; s++) {
						double val = 0.0;
						
						for(int sPrime=0; sPrime<nStates; sPrime++) {
							val += pomdp.getObservationProbability(a, sPrime, o) * pomdp.getTransitionProbability(s, a, sPrime) * V.get(k).getEntry(sPrime);
						}
						
						entries[s] = val;
					}
					
					AlphaVector av = new AlphaVector(entries);
					av.setAction(a);
					gkao[k][a][o] = av;
				}
			}
		}
		assert gkao.length == V.size();
		
		// run the backup stage
		while(Btilde.size() > 0) {
			// sample a belief point uniformly at random
			int beliefIndex = rnd.nextInt(Btilde.size());
			BeliefPoint b = Btilde.get(beliefIndex);
			
			// compute backup(b)
			AlphaVector alpha = backup(pomdp, immediateRewards, gkao, b);
			
			// check if we need to add alpha
			double oldValue = AlphaVector.getValue(b.getBelief(), V);
			double newValue = alpha.getDotProduct(b.getBelief());
			
			if(newValue >= oldValue) {
				assert alpha.getAction() >= 0 && alpha.getAction() < pomdp.getNumActions() : "invalid action: "+alpha.getAction();
				Vnext.add(alpha);
			}
			else {
				int bestVectorIndex = AlphaVector.getBestVectorIndex(b.getBelief(), V);
				assert V.get(bestVectorIndex).getAction() >= 0 && V.get(bestVectorIndex).getAction() < pomdp.getNumActions() : "invalid action: "+V.get(bestVectorIndex).getAction();
				Vnext.add(V.get(bestVectorIndex));
			}
			
			// compute new Btilde containing non-improved belief points
			List<BeliefPoint> newBtilde = new ArrayList<BeliefPoint>();			
			for(BeliefPoint bp : B) {
				double oV = AlphaVector.getValue(bp.getBelief(), V);
				double nV = AlphaVector.getValue(bp.getBelief(), Vnext);
				
				if(nV < oV) {
					newBtilde.add(bp);
				}
			}
			
			Btilde = newBtilde;
		}
		
		return Vnext;
	}
	
	private AlphaVector backup(POMDP pomdp, List<AlphaVector> immediateRewards, AlphaVector[][][] gkao, BeliefPoint b) {
		int nStates = pomdp.getNumStates();
		int nActions = pomdp.getNumActions();
		int nObservations = pomdp.getNumObservations();
		
		List<AlphaVector> ga = new ArrayList<AlphaVector>();
		
		for(int a=0; a<nActions; a++) {
			List<AlphaVector> oVectors = new ArrayList<AlphaVector>();
			for(int o=0; o<nObservations; o++) {
				double maxVal = Double.NEGATIVE_INFINITY;
				AlphaVector maxVector = null;
				
				int K = gkao.length;
				for(int k=0; k<K; k++) {
					double product = gkao[k][a][o].getDotProduct(b.getBelief());
					if(product > maxVal) {
						maxVal = product;
						maxVector = gkao[k][a][o];
					}
				}
				
				assert maxVector != null;
				oVectors.add(maxVector);
			}
			
			assert oVectors.size() > 0;
			
			// take sum of the vectors
			AlphaVector sumVector = oVectors.get(0);
			for(int j=1; j<oVectors.size(); j++) {
				sumVector = AlphaVector.sumVectors(sumVector, oVectors.get(j));
			}
			
			// multiply by discount factor
			double[] sumVectorEntries = sumVector.getEntries();
			for(int s=0; s<nStates; s++) {
				sumVectorEntries[s] = pomdp.getDiscountFactor() * sumVectorEntries[s];
			}
			sumVector.setEntries(sumVectorEntries);
			
			AlphaVector av = AlphaVector.sumVectors(immediateRewards.get(a), sumVector);
			av.setAction(a);
			ga.add(av);
		}
		
		assert ga.size() == nActions;
		
		// find the maximizing vector
		double maxVal = Double.NEGATIVE_INFINITY;
		AlphaVector vFinal = null;
		for(AlphaVector av : ga) {
			double product = av.getDotProduct(b.getBelief());
			if(product > maxVal) {
				maxVal = product;
				vFinal = av;
			}
		}
		assert vFinal != null;
		
		return vFinal;
	}

	public ArrayList<AlphaVector> solve(POMDP pomdp) {		
		int nStates = pomdp.getNumStates();
		int nActions = pomdp.getNumActions();
		
		System.out.println();
		System.out.println("=== RUN POMDP SOLVER ===");
		System.out.println("Algorithm: Perseus (point-based value iteration)");
		System.out.println("Belief sampling started...");
		
		ArrayList<BeliefPoint> B = getBeliefPoints(pomdp);
		System.out.println("Number of beliefs: "+B.size());
		System.out.println();
		
		// create initial vector set and vectors defining immediate rewards
		ArrayList<AlphaVector> V = new ArrayList<AlphaVector>();
		ArrayList<AlphaVector> immediateRewards = new ArrayList<AlphaVector>();
		for(int a=0; a<nActions; a++) {
			double[] entries = new double[nStates];
			for(int s=0; s<nStates; s++) {
				entries[s] = pomdp.getReward(s, a);
			}
			AlphaVector av = new AlphaVector(entries);
			av.setAction(a);
			V.add(av);
			immediateRewards.add(av);
		}
		
		int stage = 1;

		System.out.println("Stage 1: "+V.size()+" vectors");
		
		OutputFileWriter.dumpValueFunction(pomdp, V, sp.getOutputDir()+"/"+pomdp.getInstanceName()+".alpha"+stage, sp.dumpActionLabels());
		
		// run the backup stages
		long startTime = System.currentTimeMillis();
		while(true) {
			stage++;
			
			ArrayList<AlphaVector> Vnext = backupStage(pomdp, immediateRewards, V, B);
			double valueDifference = getValueDifference(B, V, Vnext);
			double elapsed = (System.currentTimeMillis() - startTime) * 0.001;
			System.out.println("Stage "+stage+": "+Vnext.size()+" vectors, diff "+valueDifference+", time elapsed "+elapsed+" sec");
			
			V = Vnext;
			
			OutputFileWriter.dumpValueFunction(pomdp, V, sp.getOutputDir()+"/"+pomdp.getInstanceName()+".alpha"+stage, sp.dumpActionLabels());
			
			double elapsedTime = (System.currentTimeMillis() - startTime) * 0.001;
			if(valueDifference < sp.getValueFunctionTolerance() || elapsedTime > sp.getTimeLimit()) {
				break;
			}
		}
		
		totalSolveTime = (System.currentTimeMillis() - startTime);
		expectedValue = AlphaVector.getValue(pomdp.getInitialBelief().getBelief(), V);
		
		String outputFileAlpha = sp.getOutputDir()+"/"+pomdp.getInstanceName()+".alpha";
		OutputFileWriter.dumpValueFunction(pomdp, V, outputFileAlpha, sp.dumpActionLabels());
		
		return V;
	}

	private double getValueDifference(List<BeliefPoint> B, ArrayList<AlphaVector> V, ArrayList<AlphaVector> Vnext) {
		double maxDifference = Double.NEGATIVE_INFINITY;
		
		for(BeliefPoint b : B) {
			double diff = AlphaVector.getValue(b.getBelief(), Vnext) - AlphaVector.getValue(b.getBelief(), V);
			if(diff > maxDifference) maxDifference = diff;
		}
		
		return maxDifference;
	}
	
	/**
	 * Get expected value of the solution
	 * @return expected value
	 */
	public double getExpectedValue() {
		return expectedValue;
	}
}
