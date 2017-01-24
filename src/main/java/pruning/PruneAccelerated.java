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
 * Algorithm: Generalized Incremental Pruning (GIP) + accelerated pruning
 * Description: Cassandra, Littman and Zhang (1997) + Walraven and Spaan (2017)
 */

public class PruneAccelerated implements PruneMethod {
	private String name = "Generalized incremental pruning with accelerated pruning";
	private LPModel lp;
	
	public ArrayList<AlphaVector> crossSum(VectorSetCollection vsc) {
		assert vsc.size() >= 2;
		
		pruneVectorSetCollection(vsc);
		
		ArrayList<AlphaVector> U = vsc.getVectorSet(0);
		ArrayList<AlphaVector> W = vsc.getVectorSet(1);
		ArrayList<AlphaVector> crossUW = AlphaVector.crossSum(U, W);
		ArrayList<AlphaVector> crossSum = pruneAfterCrossSum(crossUW, U, W);
		
		for(int i=2; i<vsc.size(); i++) {
			U = crossSum;
			W = vsc.getVectorSet(i);
			crossUW = AlphaVector.crossSum(U, W);
			crossSum = pruneAfterCrossSum(crossUW, U, W);
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
	
	private ArrayList<AlphaVector> pruneAfterCrossSum(ArrayList<AlphaVector> vectors, ArrayList<AlphaVector> U, ArrayList<AlphaVector> W) {
		ArrayList<AlphaVector> Q = new ArrayList<AlphaVector>(vectors);
		ArrayList<AlphaVector> D = new ArrayList<AlphaVector>();
		
		while(Q.size() > 0) {			
			int zIndex = 0;
			AlphaVector z = Q.get(zIndex);
			
			if(z.isPointwiseDominated(D)) {
				Q.remove(zIndex);
			}
			else {
				int uOrigin = z.getOriginU();
				int wOrigin = z.getOriginW();
				
				// determine size of potential D' and D'' sets
				int d1count = 1 * (W.size()-1);
				int d2count = (U.size()-1) * 1;
				
				for(AlphaVector av : D) {
					if(av.getOriginU() != uOrigin && av.getOriginW() == wOrigin) {
						d1count++;
					}
					
					if(av.getOriginW() != wOrigin && av.getOriginU() == uOrigin) {
						d2count++;
					}
				}
				
				// use either D, D' or D'' to obtain b
				double[] b = null;
				
				if(d1count < D.size() && d1count < d2count) {
					ArrayList<AlphaVector> D1 = AlphaVector.crossSumRestricted(U.get(uOrigin), W, wOrigin);
					
					for(AlphaVector av : D) {
						if(av.getOriginU() != uOrigin && av.getOriginW() == wOrigin) {
							D1.add(av);
						}
					}
					
					b = lp.findRegionPointAccelerated(z,D1);
					
				}
				else if(d2count < D.size() && d2count < d1count) {
					ArrayList<AlphaVector> D2 = AlphaVector.crossSumRestricted(W.get(wOrigin), U, uOrigin);
					
					for(AlphaVector av : D) {
						if(av.getOriginW() != wOrigin && av.getOriginU() == uOrigin) {
							D2.add(av);
						}
					}
					
					b = lp.findRegionPointAccelerated(z,D2);
				}
				else {
					b = lp.findRegionPointAccelerated(z,D);
				}
				
				
				if(b == null) {
					Q.remove(zIndex);
				}
				else {
					zIndex = AlphaVector.getBestVectorIndex(b, Q);
					z = Q.get(zIndex);
					D.add(z);
					Q.remove(zIndex);
				}
			}
		}
		
		return D;
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
				double[] b = lp.findRegionPointAccelerated(w,D);
				
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
