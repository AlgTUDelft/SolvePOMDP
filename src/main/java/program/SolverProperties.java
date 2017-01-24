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

package program;

public class SolverProperties {
	private int fixedStages = -1;               // fixed number of stages, and -1 otherwise
	private double epsilon;                     // vectors are included if d>epsilon
	private double valueFunctionTolerance;      // allowed bellman error
	private int acceleratedLPThreshold;         // accelerated LP is only used if |U| > threshold
	private double acceleratedLPTolerance;      // will be used to determine when the accelerated LP routine terminates
	private double coefficientThreshold;        // if absolute value of an LP coefficient is lower than threshold, it will be set to zero
	private boolean dumpPolicyGraph;            // if true, then the solver writes a policy graph to a file
	private boolean dumpActionLabels;			// if true, then the solver writes action labels rather than IDs
	private String workingDir;                  // path of the working directory (empty if executed from IDE)
	private String outputDirName;               // name of the output directory, which should be a directory in workingDir
	private double timeLimit;                   // time limit in seconds
	private int beliefSamplingRuns;             // belief sampling runs
	private int beliefSamplingSteps;            // belief sampling steps
	
	public int getFixedStages() {
		return fixedStages;
	}
	
	public void setFixedStages(int fixedStages) {
		this.fixedStages = fixedStages;
	}
	
	public double getEpsilon() {
		return epsilon;
	}
	
	public void setEpsilon(double epsilon) {
		this.epsilon = epsilon;
	}
	
	public double getValueFunctionTolerance() {
		return valueFunctionTolerance;
	}
	
	public void setValueFunctionTolerance(double valueFunctionTolerance) {
		this.valueFunctionTolerance = valueFunctionTolerance;
	}

	public int getAcceleratedLPThreshold() {
		return acceleratedLPThreshold;
	}

	public void setAcceleratedLPThreshold(int bendersThreshold) {
		this.acceleratedLPThreshold = bendersThreshold;
	}

	public double getAcceleratedLPTolerance() {
		return acceleratedLPTolerance;
	}

	public void setAcceleratedLPTolerance(double bendersTolerance) {
		this.acceleratedLPTolerance = bendersTolerance;
	}

	public double getCoefficientThreshold() {
		return coefficientThreshold;
	}

	public void setCoefficientThreshold(double coefficientThreshold) {
		this.coefficientThreshold = coefficientThreshold;
	}

	public boolean dumpPolicyGraph() {
		return dumpPolicyGraph;
	}

	public void setDumpPolicyGraph(boolean dumpPolicyGraph) {
		this.dumpPolicyGraph = dumpPolicyGraph;
	}

	public boolean dumpActionLabels() {
		return dumpActionLabels;
	}

	public void setDumpActionLabels(boolean dumpActionLabels) {
		this.dumpActionLabels = dumpActionLabels;
	}

	public String getOutputDirName() {
		return outputDirName;
	}

	public void setOutputDirName(String outputDirName) {
		this.outputDirName = outputDirName;
	}

	public String getOutputDir() {
		if(workingDir.length() == 0) {
			return outputDirName;
		}
		else {
			return workingDir+"/"+outputDirName;
		}
	}

	public String getWorkingDir() {
		return workingDir;
	}

	public void setWorkingDir(String workingDir) {
		this.workingDir = workingDir;
	}

	public double getTimeLimit() {
		return timeLimit;
	}

	public void setTimeLimit(double timeLimit) {
		this.timeLimit = timeLimit;
	}

	public int getBeliefSamplingRuns() {
		return beliefSamplingRuns;
	}

	public void setBeliefSamplingRuns(int beliefSamplingRuns) {
		this.beliefSamplingRuns = beliefSamplingRuns;
	}

	public int getBeliefSamplingSteps() {
		return beliefSamplingSteps;
	}

	public void setBeliefSamplingSteps(int beliefSamplingSteps) {
		this.beliefSamplingSteps = beliefSamplingSteps;
	}
}
