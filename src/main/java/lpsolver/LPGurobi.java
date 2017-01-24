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

import gurobi.GRB;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBVar;

import java.util.ArrayList;

import solver.AlphaVector;

public class LPGurobi implements LPModel {
	private GRBEnv env;
	
	private double epsilon = 0.000001;
	private int acceleratedLPThreshold = 200;
	private double acceleratedLPTolerance = 0.0001;
	private double coefficientThreshold = 0.0;
	
	public LPGurobi() {
		
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
			GRBModel model = new GRBModel(env);
			
			// create variables
			GRBVar dVar = model.addVar(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0, GRB.CONTINUOUS, "d");
			
			GRBVar[] bVar = new GRBVar[nStates];
			for(int i=0; i<nStates; i++) {
				bVar[i] = model.addVar(0.0, 1.0, 0.0, GRB.CONTINUOUS, "b_"+i);
			}
			
			model.update();
			
			// set objective
			GRBLinExpr expr = new GRBLinExpr();
			expr.addTerm(1.0, dVar);
			model.setObjective(expr, GRB.MAXIMIZE);
			
			// add constraint for belief vector
			expr = new GRBLinExpr();
			for(int i=0; i<nStates; i++) {
				expr.addTerm(1.0, bVar[i]);
			}
			model.addConstr(expr, GRB.EQUAL, 1.0, "b_constr");
			
			// add a constraint for each vector
			double[] wEntries = w.getEntries();
			
			for(AlphaVector u : U) {
				expr = new GRBLinExpr();
				double[] uEntries = u.getEntries();
				for(int i=0; i<nStates; i++) {
					expr.addTerm(getCoefficient(wEntries[i] - uEntries[i]), bVar[i]);
				}
				expr.addTerm(getCoefficient(-1.0), dVar);
				model.addConstr(expr, GRB.GREATER_EQUAL, 0.0, "v_constr");
			}
			
			model.update();
			model.optimize();
			
			int status = model.get(GRB.IntAttr.Status);
			
			if(status == GRB.Status.OPTIMAL) {
				double d = dVar.get(GRB.DoubleAttr.X);
				double objective = model.get(GRB.DoubleAttr.ObjVal);
				
				if(d > epsilon && objective > epsilon) {
					b = new double[nStates];
					for(int i=0; i<nStates; i++) {
						b[i] = bVar[i].get(GRB.DoubleAttr.X);
					}
				}
			}
			
			model.dispose();
		}
		catch (GRBException e) {
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
			GRBModel model = new GRBModel(env);

			// create variables
			GRBVar thetaVar = model.addVar(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0, GRB.CONTINUOUS, "theta");

			GRBVar[] bVar = new GRBVar[nStates];
			for (int i = 0; i < nStates; i++) {
				bVar[i] = model.addVar(0.0, 1.0, 0.0, GRB.CONTINUOUS, "b_" + i);
			}

			model.update();

			// set objective
			GRBLinExpr expr = new GRBLinExpr();
			expr.addTerm(1.0, thetaVar);
			model.setObjective(expr, GRB.MAXIMIZE);

			// add constraint for belief vector
			expr = new GRBLinExpr();
			for (int i = 0; i < nStates; i++) {
				expr.addTerm(1.0, bVar[i]);
			}
			model.addConstr(expr, GRB.EQUAL, 1.0, "b_constr");
			
			// select initial theta constraint			
			double[] tmpB = new double[nStates];
			tmpB[0] = 1.0;
			int k = selectConstraintVector(U, w, tmpB);
					
			double currentMax = Double.MAX_VALUE;
			double[] b = null;
			double[] lastB = new double[nStates];
			
			int nConstraints = 0;
			boolean[] constraintAdded = new boolean[U.size()];
			constraintAdded[k] = true;
			
			while(true) {
				// add new constraint
				AlphaVector kVector = U.get(k);
				double[] kVectorEntries = kVector.getEntries();
				
				expr = new GRBLinExpr();
				expr.addTerm(getCoefficient(1.0), thetaVar);
				for (int i = 0; i < nStates; i++) {
					expr.addTerm(getCoefficient(-1.0 * (wEntries[i] - kVectorEntries[i])), bVar[i]);
				}
				model.addConstr(expr, GRB.LESS_EQUAL, 0.0, "theta_constr");
				
				
				nConstraints++;

				// solve the model again
				model.update();
				model.optimize();
				
				// obtain new belief
				b = new double[nStates];
				double beliefDiff = 0.0;
				for (int i = 0; i < nStates; i++) {
					b[i] = bVar[i].get(GRB.DoubleAttr.X);
					beliefDiff += Math.abs(lastB[i]-b[i]);
				}
				
				lastB = b;
				
				// get current objective
				double currentObjective = thetaVar.get(GRB.DoubleAttr.X);
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
					retB[i] = bVar[i].get(GRB.DoubleAttr.X);
				}
			}

			model.dispose();
		} catch (GRBException e) {
			e.printStackTrace();
		}
		
		return retB;
	}
	
	// The method below is identical to findRegionPoint, but is called from the accelerated LP method if U is small
	
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
			GRBModel model = new GRBModel(env);
			
			// create variables
			GRBVar dVar = model.addVar(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0, GRB.CONTINUOUS, "d");
			
			GRBVar[] bVar = new GRBVar[nStates];
			for(int i=0; i<nStates; i++) {
				bVar[i] = model.addVar(0.0, 1.0, 0.0, GRB.CONTINUOUS, "b_"+i);
			}
			
			model.update();
			
			// set objective
			GRBLinExpr expr = new GRBLinExpr();
			expr.addTerm(1.0, dVar);
			model.setObjective(expr, GRB.MAXIMIZE);
			
			// add constraint for belief vector
			expr = new GRBLinExpr();
			for(int i=0; i<nStates; i++) {
				expr.addTerm(1.0, bVar[i]);
			}
			model.addConstr(expr, GRB.EQUAL, 1.0, "b_constr");
			
			// add a constraint for each vector
			double[] wEntries = w.getEntries();
			
			for(AlphaVector u : U) {
				expr = new GRBLinExpr();
				double[] uEntries = u.getEntries();
				for(int i=0; i<nStates; i++) {
					expr.addTerm(getCoefficient(wEntries[i] - uEntries[i]), bVar[i]);
				}
				expr.addTerm(getCoefficient(-1.0), dVar);
				model.addConstr(expr, GRB.GREATER_EQUAL, 0.0, "v_constr");
			}
			
			model.update();
			model.optimize();
			
			int status = model.get(GRB.IntAttr.Status);
			
			if(status == GRB.Status.OPTIMAL) {
				double d = dVar.get(GRB.DoubleAttr.X);
				double objective = model.get(GRB.DoubleAttr.ObjVal);
				
				if(d > epsilon && objective > epsilon) {
					b = new double[nStates];
					for(int i=0; i<nStates; i++) {
						b[i] = bVar[i].get(GRB.DoubleAttr.X);
					}
				}
			}
			
			model.dispose();
		}
		catch (GRBException e) {
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
			GRBModel model = new GRBModel(env);
			
			// create variables
			GRBVar dVar = model.addVar(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0, GRB.CONTINUOUS, "d");
			
			GRBVar[] bVar = new GRBVar[nStates];
			for(int i=0; i<nStates; i++) {
				bVar[i] = model.addVar(0.0, 1.0, 0.0, GRB.CONTINUOUS, "b_"+i);
			}
			
			model.update();
			
			// set objective
			GRBLinExpr expr = new GRBLinExpr();
			expr.addTerm(1.0, dVar);
			model.setObjective(expr, GRB.MAXIMIZE);
			
			// add constraint for belief vector
			expr = new GRBLinExpr();
			for(int i=0; i<nStates; i++) {
				expr.addTerm(1.0, bVar[i]);
			}
			model.addConstr(expr, GRB.EQUAL, 1.0, "b_constr");
			
			// add a constraint for each vector
			double[] wEntries = w.getEntries();
			
			for(AlphaVector u : U) {
				expr = new GRBLinExpr();
				double[] uEntries = u.getEntries();
				for(int i=0; i<nStates; i++) {
					expr.addTerm(getCoefficient(wEntries[i] - uEntries[i]), bVar[i]);
				}
				expr.addTerm(getCoefficient(-1.0), dVar);
				model.addConstr(expr, GRB.GREATER_EQUAL, 0.0, "v_constr");
			}
			
			model.update();
			model.optimize();
			
			double d = dVar.get(GRB.DoubleAttr.X);
			
			if(d > 0) {
				retD = d;
			}
			
			model.dispose();
		}
		catch (GRBException e) {
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
		try {
			env = new GRBEnv();
			env.set(GRB.DoubleParam.MIPGap, 0.00001);
			env.set(GRB.IntParam.OutputFlag, 0);
		}
		catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public void close() {
		try {
			env.dispose();
		}
		catch (GRBException e) {
			e.printStackTrace();
		}
	}
	
	public String getName() {
		return "Gurobi";
	}
}
