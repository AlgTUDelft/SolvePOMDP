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
import program.POMDP;

public interface Solver {
	/**
	 * Get the type of algorithm used by the solver: exact or approximate
	 * @return solver type
	 */
	public String getType();
	
	/**
	 * Get the total solving time in seconds
	 * @return running time in seconds
	 */
	public double getTotalSolveTime();
	
	/**
	 * Solve a POMDP and return a list containing alphavectors
	 * @param pomdp POMDP model
	 * @return alphavectors representing solution
	 */
	public ArrayList<AlphaVector> solve(POMDP pomdp);
	

	/**
	 * Get expected value of the solution
	 * @return expected value
	 */
	public double getExpectedValue();
}
