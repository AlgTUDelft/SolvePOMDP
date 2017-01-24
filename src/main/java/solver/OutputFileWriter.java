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

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;

import program.POMDP;

public class OutputFileWriter {
	/**
	 * Get the string of the floating point number d with 17 decimal digits
	 * @param d floating point number
	 * @return string corresponding to d
	 */
	private static String doubleToString(double d) {
		// 17 is the maximum number of decimals that can be represented in IEEE 754 double floating point (varies from 15 to 17)
		return String.format("%.17f", d);
	}
	
	/**
	 * Write a value function to a file
	 * @param vectors vector set representing the value function
	 * @param outputFile output file where values should be written
	 */
	public static void dumpValueFunction(POMDP pomdp, ArrayList<AlphaVector> vectors, String outputFile, boolean useActionLabels) {
		try {
			Writer output = new BufferedWriter(new FileWriter(outputFile));
			
			for(AlphaVector a : vectors) {
				String actionLabel = useActionLabels ? pomdp.getActionLabel(a.getAction()) : a.getAction()+"";
				
				output.write(actionLabel+"\n");
				//assert a.getAction() != -1;
				
				for(int i=0; i<a.size(); i++) {
					output.write(doubleToString(a.getEntry(i)) + " ");
				}
				
				output.write("\n\n");
			}
			
			output.close();
		}
		catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Write a policy graph to a file
	 * @param vectors vector set representing the value function and policy graph information
	 * @param outputFile output file where policy graph should be written
	 */
	public static void dumpPolicyGraph(POMDP pomdp, ArrayList<AlphaVector> vectors, String outputFile, boolean useActionLabels) {
		// some initial computations, because we want to know which action-observation pairs are possible
		boolean[][] observationPossible = new boolean[pomdp.getNumActions()][pomdp.getNumObservations()];
		for(int a=0; a<pomdp.getNumActions(); a++) {
			for(int o=0; o<pomdp.getNumObservations(); o++) {
				boolean pairPossible = false;
				
				for(int s=0; s<pomdp.getNumStates() && !pairPossible; s++) {
					if(pomdp.getObservationProbability(a, s, o) > 0.0) {
						pairPossible = true;
					}
				}
				
				observationPossible[a][o] = pairPossible;
			}
		}
		
		// write the policy graph to a file
		try {
			Writer output = new BufferedWriter(new FileWriter(outputFile));
			
			for(int i=0; i<vectors.size(); i++) {
				AlphaVector av = vectors.get(i);
				int action = av.getAction();
				String actionLabel = useActionLabels ? pomdp.getActionLabel(action) : av.getAction()+"";
				
				String obsNodes = "";
				int[] obsNodeIDs = av.getObsSource();
				for(int o=0; o<obsNodeIDs.length; o++) {
					String observationLabel = observationPossible[action][o] ? obsNodeIDs[o]+"" : "-";
					obsNodes += observationLabel+" ";
				}
				
				output.write(i+" "+actionLabel+" "+obsNodes);
				output.write("\n");
			}
			
			output.close();
		}
		catch (IOException e) {
			e.printStackTrace();
		}
	}
}
