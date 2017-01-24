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

package simulator;

import java.util.Random;

import program.POMDP;
import program.Parser;
import solver.BeliefPoint;
import solver.ProbabilitySample;

public class PolicySimulator {
	private POMDP pomdp;
	private Random rnd;
	private Policy policy;
	
	private boolean simulationDone = false;
	private double meanDiscountedValue;
	
	public PolicySimulator(POMDP pomdp, Policy policy, Random rnd) {
		assert pomdp != null && rnd != null;
		this.pomdp = pomdp;
		this.rnd = rnd;
		this.policy = policy;
	}
	
	public void run(int runs, int steps) {
		double totalValue = 0.0;
		
		for(int run=0; run<runs; run++) {
			if(run % 5000 == 0) System.out.println(run);
			
			BeliefPoint b = pomdp.getInitialBelief();
				
			// sample an initial state
			ProbabilitySample ps = new ProbabilitySample(rnd);
			for(int s=0; s<pomdp.getNumStates(); s++) {
				ps.addItem(s, b.getBelief(s));
			}
			int state = ps.sampleItem();
				
			// reset the policy before we start
			policy.reset();
				
			double runValue = 0.0;
			for(int step=0; step<steps; step++) {					
				// select an action
				int action = policy.getAction(b);
				assert action >= 0 && action < pomdp.getNumActions() : "invalid action: "+action;
				
				runValue += Math.pow(pomdp.getDiscountFactor(), (double) step) * pomdp.getReward(state, action);
				
				// sample next state
				ps = new ProbabilitySample(rnd);
				for(int s=0; s<pomdp.getNumStates(); s++) {
					ps.addItem(s, pomdp.getTransitionProbability(state, action, s));
				}
				int stateNext = ps.sampleItem();
				assert stateNext >= 0 && stateNext < pomdp.getNumStates();
				
				// sample an observation
				ps = new ProbabilitySample(rnd);
				for(int o=0; o<pomdp.getNumObservations(); o++) {
					double probability = pomdp.getObservationProbability(action, stateNext, o);
					ps.addItem(o, probability);
					
				}
				int observation = ps.sampleItem();
				assert observation >= 0 && observation < pomdp.getNumObservations();
				
				// tell the policy what we did
				policy.update(action, observation);
				
				// update the belief state
				BeliefPoint bao = pomdp.updateBelief(b, action, observation);
				
				// prepare for next step
				state = stateNext;
				b = bao;
			}
				
			totalValue += runValue;
		}
		
		meanDiscountedValue = totalValue / ((double) runs);
		simulationDone = true;
	}
	
	public double getMeanDiscountedValue() {
		assert simulationDone;
		return meanDiscountedValue;
	}
	
	public static void main(String[] args) {
		POMDP pomdp = Parser.readPOMDP("domains/partpainting.POMDP");
		PolicyVector policyVector = PolicyVector.readFile("output/partpainting.alpha");
		PolicyFSC policyFSC = PolicyFSC.createFSC(pomdp, "output/partpainting.alpha", "output/partpainting.pg");
		
		PolicySimulator simVector = new PolicySimulator(pomdp, policyVector, new Random(4353));
		simVector.run(50000, 200);
		
		PolicySimulator simGraph = new PolicySimulator(pomdp, policyFSC, new Random(234));
		simGraph.run(50000, 200);
		
		System.out.println("Expected value vector: "+simVector.getMeanDiscountedValue());
		System.out.println("Expected value graph: "+simGraph.getMeanDiscountedValue());
	}
	
}
