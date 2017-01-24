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

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

import solver.AlphaVector;
import solver.BeliefPoint;

public class PolicyVector implements Policy {
	private ArrayList<AlphaVector> vectors;
	
	public PolicyVector(ArrayList<AlphaVector> vectors) {
		this.vectors = vectors;
	}
	
	public int getAction(BeliefPoint b) {
		int vectorIndex = AlphaVector.getBestVectorIndex(b.getBelief(), vectors);
		assert vectorIndex >= 0 && vectorIndex < vectors.size();
		return vectors.get(vectorIndex).getAction();
	}

	public void update(int a, int o) {
		// dummy
	}

	public void reset() {
		// dummy
	}
	
	public static PolicyVector readFile(String file) {
		ArrayList<AlphaVector> vectors = new ArrayList<AlphaVector>();
		
		try {
			Scanner sc = new Scanner(new File(file));
			
			while(sc.hasNextInt()) {
				// get action
				int action = sc.nextInt();
				sc.nextLine();
				
				// get line containing doubles
				String line = sc.nextLine();
				String[] lineSplit = line.split(" ");
				double[] vectorEntries = new double[lineSplit.length];
				
				for(int i=0; i<lineSplit.length; i++) {
					vectorEntries[i] = Double.parseDouble(lineSplit[i]);
				}
				
				AlphaVector av = new AlphaVector(vectorEntries);
				av.setAction(action);
				vectors.add(av);
			}
			
			sc.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}		
		
		return new PolicyVector(vectors);
	}
}
