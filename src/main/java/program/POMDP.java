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

package program;

import java.util.HashMap;

import solver.BeliefPoint;

public class POMDP {	
	private String filename;
	private String instanceName;
	private int nStates;
	private int nActions;
	private int nObservations;
	private double discountFactor;
	
	private double[][] rewardFunction;
	private double[][][] transitionFunction;
	private double[][][] observationFunction;
	private double minReward = Double.POSITIVE_INFINITY;
	
	private BeliefPoint b0;
	
	private HashMap<Integer,String> actionLabels;
	
	public POMDP(String filename, int nStates, int nActions, int nObservations, double discountFactor, double[][] rewardFunction, double[][][] transitionFunction, double[][][] observationFunction, HashMap<Integer,String> actionLabels, BeliefPoint b0) {		
		String[] filenameSplit = filename.split("/");
		this.filename = filenameSplit[filenameSplit.length-1];
		this.instanceName = filenameSplit[filenameSplit.length-1].replace(".POMDP", "");
		this.nStates = nStates;
		this.nActions = nActions;
		this.nObservations = nObservations;
		this.discountFactor = discountFactor;
		this.rewardFunction = rewardFunction;
		this.transitionFunction = transitionFunction;
		this.observationFunction = observationFunction;
		this.actionLabels = actionLabels;
		this.b0 = b0;
		
		// compute min reward
		for(int s=0; s<nStates; s++) {
			for(int a=0; a<nActions; a++) {
				minReward = Math.min(minReward, rewardFunction[s][a]);
			}
		}
	}
	
	public int getNumStates() {
		return nStates;
	}
	
	public int getNumActions() {
		return nActions;
	}
	
	public int getNumObservations() {
		return nObservations;
	}
	
	public double getDiscountFactor() {
		return discountFactor;
	}
	
	public double getTransitionProbability(int s, int a, int sNext) {
		assert s<nStates && a<nActions && sNext<nStates;
		return transitionFunction[s][a][sNext];
	}
	
	public double getReward(int s, int a) {
		assert s<nStates && a<nActions;
		return rewardFunction[s][a];
	}
	
	public double getObservationProbability(int a, int sNext, int o) {
		assert a<nActions && sNext<nStates && o<nObservations;
		return observationFunction[a][sNext][o];
	}
	
	public double getMinReward() {
		return minReward;
	}
	
	public String getFilename() {
		return filename;
	}
	
	public String getInstanceName() {
		return instanceName;
	}
	
	public String getActionLabel(int a) {
		return actionLabels.get(a);
	}
	
	public BeliefPoint updateBelief(BeliefPoint b, int a, int o) {
		assert a<nActions && o<nObservations;
		double[] newBelief = new double[nStates];
		
		// check if belief point has been prepared
		if(!b.hasActionObservationProbabilities()) {
			prepareBelief(b);
		}
		
		// compute normalizing constant
		double nc = b.getActionObservationProbability(a, o);
		assert nc > 0.0 : "o cannot be observed when executing a in belief b";
		
		// compute the new belief vector
		for(int sNext=0; sNext<nStates; sNext++) {
			double beliefEntry = 0.0;
			
			for(int s=0; s<nStates; s++) {
				beliefEntry += getTransitionProbability(s, a, sNext) * b.getBelief(s);
			}
			
			newBelief[sNext] = beliefEntry * (getObservationProbability(a, sNext, o) / nc);
		}
		
		return new BeliefPoint(newBelief);
	}
	
	public void prepareBelief(BeliefPoint b) {
		assert b != null;
		if(b.hasActionObservationProbabilities()) return;
		
		double[][] aoProbs = new double[nActions][nObservations];
		
		for(int a=0; a<nActions; a++) {
			for(int o=0; o<nObservations; o++) {
				double prob = 0.0;
				
				for(int sNext=0; sNext<nStates; sNext++) {
					double p = 0.0;
					
					for(int s=0; s<nStates; s++) {
						p += getTransitionProbability(s, a, sNext) * b.getBelief(s);
					}
					
					prob += getObservationProbability(a, sNext, o) * p;
				}
				
				aoProbs[a][o] = prob;
			}
		}
		
		b.setActionObservationProbabilities(aoProbs);
	}
	
	public BeliefPoint getInitialBelief() {
		return b0;
	}
}
