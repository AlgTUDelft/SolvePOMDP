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

import program.POMDP;

import solver.AlphaVector;
import solver.BeliefPoint;

public class PolicyFSC implements Policy {
	private int numNodes;
	private int initialNode;
	private int[] actions;
	private int[][] nextNodes;
	
	private int currentNode;
	
	public PolicyFSC(int numNodes, int initialNode, int[] actions, int[][] nextNodes) {
		this.numNodes = numNodes;
		this.initialNode = initialNode;
		this.actions = actions;
		this.nextNodes = nextNodes;
		this.currentNode = initialNode;
	}

	public int getAction(BeliefPoint b) {
		// we ignore the given belief point
		return actions[currentNode];
	}

	public void update(int a, int o) {		
		currentNode = nextNodes[currentNode][o];
		assert currentNode >= 0 && currentNode < numNodes;
	}

	public void reset() {
		// reset to initial node
		currentNode = initialNode;
	}
	
	public static PolicyFSC createFSC(POMDP pomdp, String vectorFile, String policyGraphFile) {
		// read vectors
		ArrayList<AlphaVector> vectors = new ArrayList<AlphaVector>();
		try {
			Scanner sc = new Scanner(new File(vectorFile));
			
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
		
		// determine initial node
		int numNodes = vectors.size();
		int initialNode = AlphaVector.getBestVectorIndex(pomdp.getInitialBelief().getBelief(), vectors);
		
		// read policy graph
		int[] actions = new int[numNodes];
		int[][] nextNodes = new int[numNodes][pomdp.getNumObservations()];
		
		try {
			Scanner sc = new Scanner(new File(policyGraphFile));
			
			for(int node=0; node<numNodes; node++) {
				String line = sc.nextLine();
				String[] lineSplit = line.split(" ");
				
				int nodeId = Integer.parseInt(lineSplit[0]);
				assert node == nodeId;
				
				int action = Integer.parseInt(lineSplit[1]);
				actions[node] = action;
				
				for(int o=0; o<pomdp.getNumObservations(); o++) {
					if(lineSplit[o+2].equals("-")) {
						nextNodes[node][o] = 0; // this transition will never be used
					}
					else {
						nextNodes[node][o] = Integer.parseInt(lineSplit[o+2]);
					}
				}
			}
			
			sc.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		
		return new PolicyFSC(numNodes, initialNode, actions, nextNodes);
	}
}
