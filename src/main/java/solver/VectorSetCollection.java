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

public class VectorSetCollection {
	ArrayList<ArrayList<AlphaVector>> collection = new ArrayList<ArrayList<AlphaVector>>();
	
	/**
	 * Add vector set to the collection
	 * @param set vector set
	 */
	public void addVectorSet(ArrayList<AlphaVector> set) {
		collection.add(set);
	}
	
	/**
	 * Get vector set with index i
	 * @param i index
	 * @return vector set i
	 */
	public ArrayList<AlphaVector> getVectorSet(int i) {
		assert i < collection.size();
		return collection.get(i);
	}
	
	/**
	 * Replace vector set i in the collection
	 * @param i index
	 * @param vectorSet new vector set
	 */
	public void setVectorSet(int i, ArrayList<AlphaVector> vectorSet) {
		collection.set(i, vectorSet);
	}
	
	/**
	 * Get number of vector sets in the collection
	 * @return size
	 */
	public int size() {
		return collection.size();
	}
}
