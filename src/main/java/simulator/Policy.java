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

import solver.BeliefPoint;

public interface Policy {
	/**
	 * Get the action that should be executed for the given belief
	 * @param b belief
	 * @return action
	 */
	public int getAction(BeliefPoint b);
	
	/**
	 * Update policy after executing a and observing o (used in finite-state controllers)
	 * @param a action
	 * @param o observation
	 */
	public void update(int a, int o);
	
	/**
	 * Reset initial state of the policy (used in finite-state controllers)
	 */
	public void reset();
}
