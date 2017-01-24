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

import lpsolve.LpSolve;
import lpsolve.LpSolveException;

public class LPSolve implements LPModel {	
	private double epsilon = 0.00000001;
	private int acceleratedLPThreshold = 200;
	private double acceleratedLPTolerance = 0.0001;
	private double coefficientThreshold = 0.000000001;
	
	public LPSolve() {

	}
	
	public double[] findRegionPoint(AlphaVector w, ArrayList<AlphaVector> U) {
		double[] b = null;
		int nStates = w.size();
		
		// if U is empty, then any b is a witness point
		if(U.size() == 0) {
			b = new double[w.size()];
			b[0] = 1.0;
			return b;
		}
		
		try {
			int nVar = nStates+1;
			LpSolve solver = LpSolve.makeLp(0, nVar);
			solver.setVerbose(0);
			
			// LPSOLVE INDICES START FROM 1 WHEN DEFINING CONSTRAINTS!
			
			// configure variables
			for(int i=1; i<=nStates; i++) {
				solver.setLowbo(i, 0.0);
				solver.setUpbo(i, 1.0);
			}
			solver.setLowbo(nStates+1, -1.0 * Double.MAX_VALUE);
			solver.setUpbo(nStates+1, Double.MAX_VALUE);
			
			// add constraint for belief vector
			double[] expr = new double[nVar+1];
			for(int i=0; i<nStates; i++) {
				expr[i+1] = 1.0;
			}
			
			expr[nStates+1] = 0.0;
			solver.addConstraint(expr, LpSolve.EQ, 1.0);

			// add a constraint for each vector
			double[] wEntries = w.getEntries();
			
			for(AlphaVector u : U) {
				expr = new double[nVar+1];
				double[] uEntries = u.getEntries();
				for(int i=0; i<nStates; i++) {
					expr[i+1] = getCoefficient(wEntries[i] - uEntries[i]);
				}
				expr[nStates+1] = getCoefficient(-1.0);
				solver.addConstraint(expr, LpSolve.GE, 0.0);
			}

			// set objective function
			expr = new double[nVar+1];
			expr[nStates+1] = 1.0;
			solver.setObjFn(expr);
			solver.setMaxim();
			
			solver.solve();
				
			double[] var = solver.getPtrVariables();
			double d = var[nStates];
			double objective = solver.getObjective();
			
			if(d > epsilon && objective > epsilon) {
				b = new double[nStates];
				for(int i=0; i<nStates; i++) {
					b[i] = var[i];
				}
			}
			
			solver.deleteLp();
		}
		catch (LpSolveException e){
			e.printStackTrace();
		}
		
		return b;
	}
	
	public double[] findRegionPointAccelerated(AlphaVector w, ArrayList<AlphaVector> U) {
		int nStates = w.size();
		double[] retB = null;
		
		// if U is empty, then any b is a witness point
		if(U.size() == 0) {
			retB = new double[w.size()];
			retB[0] = 1.0;
			return retB;
		}
		
		// call regular algorithm if U is too small
		if(U.size() <= acceleratedLPThreshold || acceleratedLPThreshold == 0) {
			retB = findRegionPointRegular(w, U);
			return retB;
		}
		
		double[] wEntries = w.getEntries();
		
		try {
			int nVar = nStates+1;
			LpSolve solver = LpSolve.makeLp(0, nVar);
			solver.setVerbose(0);
			
			// LPSOLVE INDICES START FROM 1 WHEN DEFINING CONSTRAINTS!
			
			// configure variables
			for(int i=1; i<=nStates; i++) {
				solver.setLowbo(i, 0.0);
				solver.setUpbo(i, 1.0);
			}
			solver.setLowbo(nStates+1, -1.0 * Double.MAX_VALUE);
			solver.setUpbo(nStates+1, Double.MAX_VALUE);
			
			// add constraint for belief vector
			double[] expr = new double[nVar+1];
			for(int i=0; i<nStates; i++) {
				expr[i+1] = 1.0;
			}
			
			expr[nStates+1] = 0.0;
			solver.addConstraint(expr, LpSolve.EQ, 1.0);
			
			// set objective function
			expr = new double[nVar+1];
			expr[nStates+1] = 1.0;
			solver.setObjFn(expr);
			solver.setMaxim();
			
			// select initial theta constraint			
			double[] tmpB = new double[nStates];
			tmpB[0] = 1.0;
			int k = selectConstraintVector(U, w, tmpB);
			
			double currentMax = Double.MAX_VALUE;
			double[] b = null;
			double[] lastB = new double[nStates];
			
			int nConstraints = 0;
			double[] var;
			boolean[] constraintAdded = new boolean[U.size()];
			constraintAdded[k] = true;
			
			while(true) {
				// add new constraint
				AlphaVector kVector = U.get(k);
				double[] kVectorEntries = kVector.getEntries();
				
				expr = new double[nVar+1];
				for(int i=0; i<nStates; i++) {
					expr[i+1] = getCoefficient(-1.0*(wEntries[i] - kVectorEntries[i]));
				}
				expr[nStates+1] = getCoefficient(1.0);
				solver.addConstraint(expr, LpSolve.LE, 0.0);
				
				nConstraints++;
				solver.solve();
				
				//obtain new belief
				var = solver.getPtrVariables();
				b = new double[nStates];
				double beliefDiff = 0.0;
				for (int i = 0; i < nStates; i++) {
					b[i] = var[i];
					beliefDiff += Math.abs(lastB[i]-b[i]);
				}
				
				lastB = b;
				
				// get current objective
				double currentObjective = solver.getObjective();
				double objectiveChange = Math.abs(currentMax-currentObjective);
				currentMax = Math.min(currentObjective, currentMax);
				
				if((beliefDiff < acceleratedLPTolerance && objectiveChange < acceleratedLPTolerance) || nConstraints == U.size()+1) {
					break;
				}
				
				// stop if objective drops below the epsilon value
				if(currentMax <= epsilon) {
					break;
				}
				
				// obtain new k
				k = selectConstraintVector(U, w, b);
				
				// stop if constraint has been added before
				if(constraintAdded[k]) {
					break;
				}
				else {
					constraintAdded[k] = true;
				}
			}
			
			if(currentMax > epsilon) {
				retB = new double[nStates];
				for(int i=0; i<nStates; i++) {
					retB[i] = var[i];
				}
			}
			
			solver.deleteLp();
		}
		catch (LpSolveException e){
			e.printStackTrace();
		}
		
		return retB;
	}
	
	private double[] findRegionPointRegular(AlphaVector w, ArrayList<AlphaVector> U) {
		double[] b = null;
		int nStates = w.size();
		
		// if U is empty, then any b is a witness point
		if(U.size() == 0) {
			b = new double[w.size()];
			b[0] = 1.0;
			return b;
		}
		
		try {
			int nVar = nStates+1;
			LpSolve solver = LpSolve.makeLp(0, nVar);
			solver.setVerbose(0);
			
			// LPSOLVE INDICES START FROM 1 WHEN DEFINING CONSTRAINTS!
			
			// configure variables
			for(int i=1; i<=nStates; i++) {
				solver.setLowbo(i, 0.0);
				solver.setUpbo(i, 1.0);
			}
			solver.setLowbo(nStates+1, -1.0 * Double.MAX_VALUE);
			solver.setUpbo(nStates+1, Double.MAX_VALUE);
			
			// add constraint for belief vector
			double[] expr = new double[nVar+1];
			for(int i=0; i<nStates; i++) {
				expr[i+1] = 1.0;
			}
			
			expr[nStates+1] = 0.0;
			solver.addConstraint(expr, LpSolve.EQ, 1.0);

			// add a constraint for each vector
			double[] wEntries = w.getEntries();
			
			for(AlphaVector u : U) {
				expr = new double[nVar+1];
				double[] uEntries = u.getEntries();
				for(int i=0; i<nStates; i++) {
					expr[i+1] = getCoefficient(wEntries[i] - uEntries[i]);
				}
				expr[nStates+1] = getCoefficient(-1.0);
				solver.addConstraint(expr, LpSolve.GE, 0.0);
			}

			// set objective function
			expr = new double[nVar+1];
			expr[nStates+1] = 1.0;
			solver.setObjFn(expr);
			solver.setMaxim();
			
			solver.solve();
				
			double[] var = solver.getPtrVariables();
			double d = var[nStates];
			double objective = solver.getObjective();
			
			if(d > epsilon && objective > epsilon) {
				b = new double[nStates];
				for(int i=0; i<nStates; i++) {
					b[i] = var[i];
				}
			}
			
			solver.deleteLp();
		}
		catch (LpSolveException e){
			e.printStackTrace();
		}
		
		return b;
	}
	
	private int selectConstraintVector(ArrayList<AlphaVector> U, AlphaVector w, double[] b) {
		double minVal = Double.POSITIVE_INFINITY;
		int minIndex = -1;
		
		double[] wEntries = w.getEntries();
		
		for(int j=0; j<U.size(); j++) {
			double currentVal = 0.0;
			AlphaVector u = U.get(j);
			double[] uEntries = u.getEntries();
			
			for(int i=0; i<b.length; i++) {
				currentVal += (wEntries[i] - uEntries[i]) * b[i];
			}
			
			if(currentVal < minVal) {
				minVal = currentVal;
				minIndex = j;
			}
		}
		
		return minIndex;
	}
	
	public double getMaxValueDiff(AlphaVector w, ArrayList<AlphaVector> U) {
		assert U.size() > 0;
		int nStates = w.size();
		double retD = 0;
		
		try {
			int nVar = nStates+1;
			LpSolve solver = LpSolve.makeLp(0, nVar);
			solver.setVerbose(0);
			
			// LPSOLVE INDICES START FROM 1 WHEN DEFINING CONSTRAINTS!
			
			// configure variables
			for(int i=1; i<nStates; i++) {
				solver.setLowbo(i, 0.0);
				solver.setUpbo(i, 1.0);
			}
			solver.setLowbo(nStates+1, -1.0 * Double.MAX_VALUE);
			solver.setUpbo(nStates+1, Double.MAX_VALUE);
			
			// add constraint for belief vector
			double[] expr = new double[nVar+1];
			for(int i=0; i<nStates; i++) {
				expr[i+1] = 1.0;
			}
			
			expr[nStates+1] = 0.0;
			solver.addConstraint(expr, LpSolve.EQ, 1.0);

			// add a constraint for each vector
			double[] wEntries = w.getEntries();
			
			for(AlphaVector u : U) {
				expr = new double[nVar+1];
				double[] uEntries = u.getEntries();
				for(int i=0; i<nStates; i++) {
					expr[i+1] = getCoefficient(wEntries[i] - uEntries[i]);
				}
				expr[nStates+1] = -1.0;
				solver.addConstraint(expr, LpSolve.GE, 0.0);
			}

			// set objective function
			expr = new double[nVar+1];
			expr[nStates+1] = 1.0;
			solver.setObjFn(expr);
			solver.setMaxim();
			solver.solve();
				
			double[] var = solver.getPtrVariables();
			double d = var[nStates];
			
			if(d > epsilon) {
				retD = d;
			}
			
			solver.deleteLp();
		}
		catch (LpSolveException e){
			e.printStackTrace();
		}
		
		return retD;
	}
	
	private double getCoefficient(double c) {
		// important: before adding a scalar here, check if all coefficients go through this function!
		
		if(Math.abs(c) < coefficientThreshold) {
			return 0.0;
		}
		else {
			return c;
		}
	}

	public void setEpsilon(double epsilon) {
		this.epsilon = epsilon;
	}
	
	public void setAcceleratedLPThreshold(int threshold) {
		this.acceleratedLPThreshold = threshold;
	}
	
	public void setCoefficientThreshold(double tolerance) {
		this.coefficientThreshold = tolerance;
	}
	
	public void setAcceleratedLPTolerance(double tolerance) {
		this.acceleratedLPTolerance = tolerance;
	}
	
	public void init() {
		System.loadLibrary("lpsolve55");
	}
	
	public void close() {
		// there is nothing to close
	}
	
	public String getName() {
		return "LPSolve";
	}
}
