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
import java.util.List;

public class BeliefPoint {
	private double[] belief;
	private boolean actionObservationProbInitialized = false;
	private double[][] aoProbs; // aoProbs[a][o] represents P(o|b,a)
	
	private List<Integer> history = new ArrayList<Integer>();
	
	public BeliefPoint(double[] belief) {
		this.belief = belief;
	}
	
	/**
	 * Get array containing belief
	 * @return belief
	 */
	public double[] getBelief() {
		return belief;
	}
	
	/**
	 * Get the belief for a specific state
	 * @param s state ID
	 * @return belief
	 */
	public double getBelief(int s) {
		assert s >= 0 && s < belief.length;
		return belief[s];
	}
	
	/**
	 * Add element to the history included in this belief point
	 * @param i history element
	 */
	public void addToHistory(int i) {
		history.add(i);
	}
	
	/**
	 * Get a list representing the full action-observation history
	 * @return history
	 */
	public List<Integer> getHistory() {
		return history;
	}
	
	/**
	 * Set the full action-observation history
	 * @param history list containing history
	 */
	public void setHistory(List<Integer> history) {
		this.history = history;
	}
	
	/**
	 * Get a copy of the history included in this beliefpoint
	 * @return history
	 */
	public List<Integer> getHistoryCopy() {
		List<Integer> newHistory = new ArrayList<Integer>();
		newHistory.addAll(history);
		return newHistory;
	}
	
	/**
	 * Get the hashcode of this beliefpoint, based on its history
	 */
	public int hashCode() {
		return history.hashCode();
	}
	
	/**
	 * Checks whether two belief points are identical based on their history list
	 */
	public boolean equals(Object o) {
		if(o instanceof BeliefPoint) {
			List<Integer> otherHistory = ((BeliefPoint) o).getHistory();
			
			if(history.size() != otherHistory.size()) {
				return false;
			}
			else {
				boolean isEqual = true;
				
				for(int i=0; i<history.size()&&isEqual; i++) {
					isEqual = isEqual && (history.get(i)==otherHistory.get(i));
				}
				
				return isEqual;
			}
			
		}
		else {
			return false;
		}
	}
	
	public String toString() {
		String ret = "<BP(";
		
		for(int i=0; i<belief.length; i++) {
			ret += belief[i]+",";
		}
		
		return ret+")>";
	}
	
	/**
	 * Returns true if action observation probabilities have been initialized
	 * @return
	 */
	public boolean hasActionObservationProbabilities() {
		return actionObservationProbInitialized;
	}
	
	/**
	 * Sets the action observation probabilities for this belief
	 * @param aoProbs action observation probabilities
	 */
	public void setActionObservationProbabilities(double[][] aoProbs) {
		assert this.aoProbs == null;
		this.aoProbs = aoProbs;
		this.actionObservationProbInitialized = true;
	}
	
	/**
	 * Get action observation probability
	 * @param a action
	 * @param o observation
	 * @return action observation probability for action a and observation o
	 */
	public double getActionObservationProbability(int a, int o) {
		assert aoProbs != null;
		return aoProbs[a][o];
	}
}
