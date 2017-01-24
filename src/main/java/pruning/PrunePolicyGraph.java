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

package pruning;

import java.util.ArrayList;

import solver.AlphaVector;
import solver.VectorSetCollection;

import lpsolver.LPModel;

/*
 * Algorithm: Incremental Pruning with policy graph tracing
 * Description: Cassandra, Littman and Zhang (1997)
 */

public class PrunePolicyGraph implements PruneMethod {
	private String name = "Incremental pruning";
	private LPModel lp;
	
	public ArrayList<AlphaVector> crossSum(VectorSetCollection vsc) {
		assert vsc.size() >= 2;
		
		// first prune individual vector sets before computing the cross sum
		pruneVectorSetCollection(vsc);
		
		// compute the cross sum and set sources for both observation 0 and 1
		int numObservations = vsc.size();
		ArrayList<AlphaVector> crossSum = AlphaVector.crossSumPolicyGraph(vsc.getVectorSet(0), vsc.getVectorSet(1), numObservations);
		crossSum = prune(crossSum);
		
		// compute the remaining parts of the cross sum and set sources for the observation considered
		for(int i=2; i<vsc.size(); i++) {
			crossSum = AlphaVector.crossSumPolicyGraph(crossSum, vsc.getVectorSet(i), numObservations);
			crossSum = prune(crossSum);
		}
		
		return crossSum;
	}

	public ArrayList<AlphaVector> mergeSets(ArrayList<ArrayList<AlphaVector>> setList) {
		ArrayList<AlphaVector> retList = new ArrayList<AlphaVector>();
		
		for(ArrayList<AlphaVector> set : setList) {
			retList.addAll(set);
		}
		
		return prune(retList);
	}

	public ArrayList<AlphaVector> prune(ArrayList<AlphaVector> vectors) {		
		ArrayList<AlphaVector> W = new ArrayList<AlphaVector>(vectors);
		ArrayList<AlphaVector> D = new ArrayList<AlphaVector>();
		
		while(W.size() > 0) {			
			int wIndex = 0;
			AlphaVector w = W.get(wIndex);
			
			if(w.isPointwiseDominated(D)) {
				W.remove(wIndex);
			}
			else {
				double[] b = lp.findRegionPoint(w,D);
				
				if(b == null) {
					W.remove(wIndex);
				}
				else {
					wIndex = AlphaVector.getBestVectorIndex(b, W);
					w = W.get(wIndex);
					D.add(w);
					W.remove(wIndex);
				}
			}
		}
		
		return D;
	}
	
	private void pruneVectorSetCollection(VectorSetCollection vsc) {
		for(int i=0; i<vsc.size(); i++) {
			ArrayList<AlphaVector> vectorSet = vsc.getVectorSet(i);
			ArrayList<AlphaVector> prunedVectorSet = prune(vectorSet);
			vsc.setVectorSet(i, prunedVectorSet);
		}
	}
	
	public String getName() {
		return name;
	}
	
	public void setLPModel(LPModel lp) {
		this.lp = lp;
	}
}
