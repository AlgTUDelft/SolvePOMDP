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

import org.apache.log4j.BasicConfigurator;

import solver.AlphaVector;

import com.joptimizer.optimizers.LPOptimizationRequest;
import com.joptimizer.optimizers.LPPrimalDualMethod;

public class LPjoptimizer implements LPModel {
	private double epsilon = 0.000001;
	private int acceleratedLPThreshold = 200;
	private double acceleratedLPTolerance = 0.0001;
	private double coefficientThreshold = 0.000000001;
	
	public double[] findRegionPoint(AlphaVector w, ArrayList<AlphaVector> U) {
		int nStates = w.size();
		
		// if U is empty, then any b is a witness point
		if(U.size() == 0) {
			double[] b = new double[w.size()];
			b[0] = 1.0;
			return b;
		}

		// define lower and upper bounds of the variables
		double[] lb = new double[nStates+1];
		double[] ub = new double[nStates+1];
		for(int s=0; s<nStates; s++) {
			lb[s] = 0.0;
			ub[s] = 1.0;
		}
		lb[nStates] = Double.MAX_VALUE * -1.0;
		ub[nStates] = Double.MAX_VALUE;
		
		// create objective function
		double[] c = new double[nStates+1];
		for(int s=0; s<nStates; s++) {
			c[s] = 0.0;
		}
		c[nStates] = -1.0;
		
		// create inequality constraints
		double[][] G = new double[U.size()][nStates+1];
		for(int i=0; i<U.size(); i++) {
			for(int s=0; s<nStates; s++) {
				G[i][s] = getCoefficient((w.getEntry(s) - U.get(i).getEntry(s)) * -1.0);
			}
			G[i][nStates] = 1.0;
		}
		double[] h = new double[U.size()];
		
		// create equality constraints, such that beliefs sum to 1
		double[][] A = new double[1][nStates+1];
		for(int s=0; s<nStates; s++) {
			A[0][s] = 1.0;
		}
		A[0][nStates] = 0.0;
		
		double[] b = new double[1];
		b[0] = 1.0;
		
		// define the optimization problem
		LPOptimizationRequest or = new LPOptimizationRequest();
		or.setC(c);
		or.setLb(lb);
		or.setUb(ub);
		or.setG(G);
		or.setH(h);
		or.setA(A);
		or.setB(b);
		
		// solve the optimization problem
		LPPrimalDualMethod opt = new LPPrimalDualMethod();
		opt.setLPOptimizationRequest(or);
		
		// check return code
		try {
			int returnCode = opt.optimize();
			assert returnCode == 0 : "Unexpected joptimizer return code";
		} catch (Exception e) {
			// this is a rather nasty solution to prevent numerical stability problems
			return null;
		}
		
		// get optimization result
		double[] sol = opt.getOptimizationResponse().getSolution();
		double[] belief = new double[nStates];
		double beliefSum = 0.0;
		for(int s=0; s<nStates; s++) {
			belief[s] = sol[s];
			beliefSum += belief[s];
		}
		assert Math.abs(beliefSum - 1.0) <= 0.00001;
		assert sol[nStates] < 99999.0 : "Unexpected improvement value";
		
		if(sol[nStates] > epsilon) {
			return belief;
		}
		else {
			return null;
		}
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
			retB = findRegionPoint(w, U);
			return retB;
		}
		
		// define lower and upper bounds of the variables
		double[] lb = new double[nStates+1];
		double[] ub = new double[nStates+1];
		for(int s=0; s<nStates; s++) {
			lb[s] = 0.0;
			ub[s] = 1.0;
		}
		lb[nStates] = Double.MAX_VALUE * -1.0;
		ub[nStates] = Double.MAX_VALUE;
		
		// create objective function
		double[] c = new double[nStates+1];
		for(int s=0; s<nStates; s++) {
			c[s] = 0.0;
		}
		c[nStates] = -1.0;
		
		// create equality constraints, such that beliefs sum to 1
		double[][] A = new double[1][nStates+1];
		for(int s=0; s<nStates; s++) {
			A[0][s] = 1.0;
		}
		A[0][nStates] = 0.0;
		
		double[] bRHS = new double[1];
		bRHS[0] = 1.0;
		
		// initialize variables used during constraint generation
		double currentMax = Double.MAX_VALUE;
		double[] b = null;
		double[] lastB = new double[nStates];
		
		boolean[] constraintAdded = new boolean[U.size()];
		ArrayList<double[]> constraints = new ArrayList<double[]>();
		
		// get first constraint
		double[] tmpB = new double[nStates];
		tmpB[0] = 1.0;
		int k = selectConstraintVector(U, w, tmpB);
				
		double[] newConstraint = new double[nStates+1];
		for(int s=0; s<nStates; s++) {
			newConstraint[s] = getCoefficient((w.getEntry(s) - U.get(k).getEntry(s)) * -1.0);
		}
		newConstraint[nStates] = 1.0;
		
		constraints.add(newConstraint);
		constraintAdded[k] = true;
		
		// start row generation
		while(true) {
			// create constraint matrix
			double[][] G = new double[constraints.size()][nStates+1];
			double[] h = new double[constraints.size()];
			for(int i=0; i<constraints.size(); i++) {
				G[i] = constraints.get(i);
			}
			
			// define the optimization problem
			LPOptimizationRequest or = new LPOptimizationRequest();
			or.setC(c);
			or.setLb(lb);
			or.setUb(ub);
			or.setG(G);
			or.setH(h);
			or.setA(A);
			or.setB(bRHS);
			
			// solve the optimization problem
			LPPrimalDualMethod opt = new LPPrimalDualMethod();
			opt.setLPOptimizationRequest(or);
			
			// check return code
			try {
				int returnCode = opt.optimize();
				assert returnCode == 0 : "Unexpected joptimizer return code";
			} catch (Exception e) {
				// this is a rather nasty solution to prevent numerical stability problems
				return null;
			}
			
			// get optimization result
			double[] sol = opt.getOptimizationResponse().getSolution();
			b = new double[nStates];
			double beliefSum = 0.0;
			double beliefDiff = 0.0;
			for(int s=0; s<nStates; s++) {
				b[s] = sol[s];
				beliefSum += b[s];
				beliefDiff += Math.abs(lastB[s]-b[s]);
			}
			assert Math.abs(beliefSum - 1.0) <= 0.00001;
			lastB = b;
			
			// get current objective
			double currentObjective = sol[nStates];
			double objectiveChange = Math.abs(currentMax-currentObjective);
			currentMax = Math.min(currentObjective, currentMax);
			
			if((beliefDiff < acceleratedLPTolerance && objectiveChange < acceleratedLPTolerance) || constraints.size() == U.size()+1) {
				break;
			}
			
			// stop if objective drops below the epsilon value
			if(currentMax <= epsilon) {
				break;
			}
			
			// obtain new k
			k = selectConstraintVector(U, w, b);
			
			// stop if constraint has been added before, add otherwise
			if(constraintAdded[k]) {
				break;
			}
			else {				
				newConstraint = new double[nStates+1];
				for(int s=0; s<nStates; s++) {
					newConstraint[s] = (w.getEntry(s) - U.get(k).getEntry(s)) * -1.0;
				}
				newConstraint[nStates] = 1.0;
				constraints.add(newConstraint);
				constraintAdded[k] = true;
			}
		}
		
		// prepare result to return
		if(currentMax > epsilon) {
			retB = new double[nStates];
			for(int i=0; i<nStates; i++) {
				retB[i] = lastB[i];
			}
		}
		
		System.out.println("Improvement findRegionPointBenders: "+currentMax);
		System.out.println("Constraints added: "+constraints.size()+" / "+U.size());
		
		return retB;
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
	
	private double getCoefficient(double c) {
		// important: before adding a scalar here, check if all coefficients go through this function!
		
		if(Math.abs(c) < coefficientThreshold) {
			return 0.0;
		}
		else {
			return c;
		}
	}

	public double getMaxValueDiff(AlphaVector w, ArrayList<AlphaVector> U) {
		int nStates = w.size();

		// define lower and upper bounds of the variables
		double[] lb = new double[nStates+1];
		double[] ub = new double[nStates+1];
		for(int s=0; s<nStates; s++) {
			lb[s] = 0.0;
			ub[s] = 1.0;
		}
		lb[nStates] = Double.MAX_VALUE * -1.0;
		ub[nStates] = Double.MAX_VALUE;
		
		// create objective function
		double[] c = new double[nStates+1];
		for(int s=0; s<nStates; s++) {
			c[s] = 0.0;
		}
		c[nStates] = -1.0;
		
		// create inequality constraints
		double[][] G = new double[U.size()][nStates+1];
		for(int i=0; i<U.size(); i++) {
			for(int s=0; s<nStates; s++) {
				G[i][s] = getCoefficient((w.getEntry(s) - U.get(i).getEntry(s)) * -1.0);
			}
			G[i][nStates] = 1.0;
		}
		double[] h = new double[U.size()];
		
		// create equality constraints, such that beliefs sum to 1
		double[][] A = new double[1][nStates+1];
		for(int s=0; s<nStates; s++) {
			A[0][s] = 1.0;
		}
		A[0][nStates] = 0.0;
		
		double[] b = new double[1];
		b[0] = 1.0;
		
		// define the optimization problem
		LPOptimizationRequest or = new LPOptimizationRequest();
		or.setC(c);
		or.setLb(lb);
		or.setUb(ub);
		or.setG(G);
		or.setH(h);
		or.setA(A);
		or.setB(b);
		
		// solve the optimization problem
		LPPrimalDualMethod opt = new LPPrimalDualMethod();
		opt.setLPOptimizationRequest(or);
		
		// check return code
		try {
			int returnCode = opt.optimize();
			assert returnCode == 0 : "Unexpected joptimizer return code";
		} catch (Exception e) {
			// this is a rather nasty solution to prevent numerical stability problems
			return Double.NEGATIVE_INFINITY;
		}
		
		// get optimization result
		double[] sol = opt.getOptimizationResponse().getSolution();
		double[] belief = new double[nStates];
		double beliefSum = 0.0;
		for(int s=0; s<nStates; s++) {
			belief[s] = sol[s];
			beliefSum += belief[s];
		}
		assert Math.abs(beliefSum - 1.0) <= 0.00001;
		
		return sol[nStates];
	}

	public void setEpsilon(double epsilon) {
		this.epsilon = epsilon;
	}

	public void setAcceleratedLPThreshold(int threshold) {
		this.acceleratedLPThreshold = threshold;
	}

	public void setCoefficientThreshold(double threshold) {
		this.coefficientThreshold = threshold;
	}

	public void setAcceleratedLPTolerance(double tolerance) {
		this.acceleratedLPTolerance = tolerance;
	}

	public void init() {
		BasicConfigurator.configure();
		System.setProperty("org.apache.commons.logging.Log", "org.apache.commons.logging.impl.NoOpLog");
	}

	public void close() {
		// there is nothing to close
	}

	public String getName() {
		return "JOptimizer";
	}
}
