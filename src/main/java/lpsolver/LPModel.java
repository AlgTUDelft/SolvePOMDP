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

package lpsolver;

import java.util.ArrayList;

import solver.AlphaVector;

public interface LPModel {
	/**
	 * Compute a belief point where vector w contributes the most after adding to U
	 * @param w vector w
	 * @param U vector set U
	 * @return belief point
	 */
	public double[] findRegionPoint(AlphaVector w, ArrayList<AlphaVector> U);
	
	/**
	 * Compute a belief point where vector w contributes the most after adding to U, using the Benders method
	 * @param w vector w
	 * @param U vector set U
	 * @return belief point
	 */
	public double[] findRegionPointAccelerated(AlphaVector w, ArrayList<AlphaVector> U);
	
	/**
	 * Compute the maximum value increase obtained when adding vector w to U
	 * @param w vector w
	 * @param U vector set U
	 * @return value increase
	 */
	public double getMaxValueDiff(AlphaVector w, ArrayList<AlphaVector> U);
	
	/**
	 * Set the epsilon value. If the gain is higher than epsilon, a vector will not be pruned
	 * @param epsilon epsilon value
	 */
	public void setEpsilon(double epsilon);
	
	/**
	 * Set the threshold which is used to decide whether the accelerated LP is used
	 * @param threshold the threshold
	 */
	public void setAcceleratedLPThreshold(int threshold);
	
	/**
	 * Set the coefficient threshold. Coefficients whose absolute value is lower will be set to zero
	 * @param threshold coefficient threshold
	 */
	public void setCoefficientThreshold(double threshold);
	
	/**
	 * Set the tolerance of the accelerated LP. If difference in two iterations is less than tolerance, the accelerated LP terminates
	 * @param tolerance the tolerance
	 */
	public void setAcceleratedLPTolerance(double tolerance);
	
	/**
	 * Initialize the LP solver
	 */
	public void init();
	
	/**
	 * Close the LP solver
	 */
	public void close();
	
	/**
	 * Get name of the LP solver
	 * @return name
	 */
	public String getName();
}
