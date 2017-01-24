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

import lpsolver.LPModel;

import solver.AlphaVector;
import solver.VectorSetCollection;

public interface PruneMethod {
	/**
	 * Computes the cross sum of the vector sets in vsc
	 * @param vsc collection containing vector sets
	 * @return cross sum
	 */
	public ArrayList<AlphaVector> crossSum(VectorSetCollection vsc);
	
	/**
	 * Merge multiple vector sets into one vector set
	 * @param set containing vector sets
	 * @return merged vector set
	 */
	public ArrayList<AlphaVector> mergeSets(ArrayList<ArrayList<AlphaVector>> setList);
	
	/**
	 * Prune a vector set by removing vectors that do not contribute to the values defined by vectors
	 * @param vectors vector set
	 * @return pruned vector set
	 */
	public ArrayList<AlphaVector> prune(ArrayList<AlphaVector> vectors);
	
	/**
	 * Get name of the pruning method
	 * @return name
	 */
	public String getName();
	
	/**
	 * Set LP model to be used by the pruning method
	 * @param lp linear programming model
	 */
	public void setLPModel(LPModel lp);
}
