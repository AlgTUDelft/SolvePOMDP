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

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.util.Properties;

import pruning.PruneStandard;
import pruning.PruneAccelerated;
import pruning.PruneMethod;
import solver.Solver;
import solver.SolverExact;

import lpsolver.LPGurobi;
import lpsolver.LPModel;
import lpsolver.LPSolve;
import lpsolver.LPjoptimizer;

public class SolvePOMDP {
	private SolverProperties sp;     // object containing user-defined properties
	private PruneMethod pm;          // pruning method used by incremental pruning
	private LPModel lp;              // linear programming solver used by incremental pruning
	private Solver solver;           // the solver that we use to solve a POMDP, which is exact or approximate
	private String domainDirName;    // name of the directory containing .POMDP files
	private String domainDir;        // full path of the domain directory
	
	public SolvePOMDP() {
		// read parameters from config file
		readConfigFile();
		
		// check if required directories exist
		configureDirectories();

		// configure LP solver
		lp.setEpsilon(sp.getEpsilon());
		lp.setAcceleratedLPThreshold(sp.getAcceleratedLPThreshold());
		lp.setAcceleratedLPTolerance(sp.getAcceleratedLPTolerance());
		lp.setCoefficientThreshold(sp.getCoefficientThreshold());
		lp.init();
	}
	
	/**
	 * Read the solver.config file. It creates a properties object and it initializes
	 * the pruning method and LP solver.
	 */
	private void readConfigFile() {
		this.sp = new SolverProperties();
		
		Properties properties = new Properties();
		
		try {
			FileInputStream file = new FileInputStream("./solver.config");
			properties.load(file);
			file.close();
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		sp.setEpsilon(Double.parseDouble(properties.getProperty("epsilon")));
		sp.setValueFunctionTolerance(Double.parseDouble(properties.getProperty("valueFunctionTolerance")));
		sp.setAcceleratedLPThreshold(Integer.parseInt(properties.getProperty("acceleratedLPThreshold")));
		sp.setAcceleratedLPTolerance(Double.parseDouble(properties.getProperty("acceleratedTolerance")));
		sp.setCoefficientThreshold(Double.parseDouble(properties.getProperty("coefficientThreshold")));
		sp.setOutputDirName(properties.getProperty("outputDirectory"));
		sp.setTimeLimit(Double.parseDouble(properties.getProperty("timeLimit")));
		sp.setBeliefSamplingRuns(Integer.parseInt(properties.getProperty("beliefSamplingRuns")));
		sp.setBeliefSamplingSteps(Integer.parseInt(properties.getProperty("beliefSamplingSteps")));
		this.domainDirName = properties.getProperty("domainDirectory");
		String algorithmType = properties.getProperty("algorithmType");
		
		if(!algorithmType.equals("perseus") && !algorithmType.equals("gip")) {
			throw new RuntimeException("Unexpected algorithm type in properties file");
		}
		
		String dumpPolicyGraphStr = properties.getProperty("dumpPolicyGraph");
		if(!dumpPolicyGraphStr.equals("true") && !dumpPolicyGraphStr.equals("false")) {
			throw new RuntimeException("Policy graph property must be either true or false");
		}
		else {
			sp.setDumpPolicyGraph(dumpPolicyGraphStr.equals("true") && algorithmType.equals("gip"));
		}
		
		String dumpActionLabelsStr = properties.getProperty("dumpActionLabels");
		if(!dumpActionLabelsStr.equals("true") && !dumpActionLabelsStr.equals("false")) {
			throw new RuntimeException("Action label property must be either true or false");
		}
		else {
			sp.setDumpActionLabels(dumpActionLabelsStr.equals("true"));
		}
		
		System.out.println();
		System.out.println("=== SOLVER PARAMETERS ===");
		System.out.println("Epsilon: "+sp.getEpsilon());
		System.out.println("Value function tolerance: "+sp.getValueFunctionTolerance());
		System.out.println("Accelerated LP threshold: "+sp.getAcceleratedLPThreshold());
		System.out.println("Accelerated LP tolerance: "+sp.getAcceleratedLPTolerance());
		System.out.println("LP coefficient threshold: "+sp.getCoefficientThreshold());
		System.out.println("Time limit: "+sp.getTimeLimit());
		System.out.println("Belief sampling runs: "+sp.getBeliefSamplingRuns());
		System.out.println("Belief sampling steps: "+sp.getBeliefSamplingSteps());
		System.out.println("Dump policy graph: "+sp.dumpPolicyGraph());
		System.out.println("Dump action labels: "+sp.dumpActionLabels());
		
		// load required LP solver
		String lpSolver = properties.getProperty("lpsolver");
		if(lpSolver.equals("gurobi")) {
			this.lp = new LPGurobi();
		}
		else if(lpSolver.equals("joptimizer")) {
			this.lp = new LPjoptimizer();
		}
		else if(lpSolver.equals("lpsolve")) {
			this.lp = new LPSolve();
		}
		else {
			throw new RuntimeException("Unexpected LP solver in properties file");
		}
		
		// load required pruning algorithm
		String pruningAlgorithm = properties.getProperty("pruningMethod");
		if(pruningAlgorithm.equals("standard")) {
			this.pm = new PruneStandard();
			this.pm.setLPModel(lp);
		}
		else if(pruningAlgorithm.equals("accelerated")) {
			this.pm = new PruneAccelerated();
			this.pm.setLPModel(lp);
		}
		else {
			throw new RuntimeException("Unexpected pruning method in properties file");
		}
		
		// load required POMDP algorithm
		if(algorithmType.equals("gip")) {
			this.solver = new SolverExact(sp, lp, pm);
		}
		else if(algorithmType.equals("perseus")) {
			this.solver = null;
		}
		else {
			throw new RuntimeException("Unexpected algorithm type in properties file");
		}
		
		System.out.println("Algorithm: "+algorithmType);
		System.out.println("LP solver: "+lp.getName());
	}
	
	/**
	 * Checks if the desired domain and output directories exist, and it sets the full path to these directories.
	 */
	private void configureDirectories() {
		String path = SolvePOMDP.class.getProtectionDomain().getCodeSource().getLocation().getPath();
		String decodedPath = "";
		
		try {
			decodedPath = URLDecoder.decode(path, "UTF-8");
		} catch (UnsupportedEncodingException e) {
			e.printStackTrace();
		}
		
		if(decodedPath.endsWith(".jar")) {
			// solver has been started from jar, so we assume that output exists in the same directory as the jar file			
			int endIndex = decodedPath.lastIndexOf("/");
			String workingDir = decodedPath.substring(0, endIndex);
			sp.setWorkingDir(workingDir);
			domainDir = workingDir+"/"+domainDirName;
		}
		else {
			// solver has not been started from jar, so we assume that output exists in the current directory
			sp.setWorkingDir("");
			domainDir = domainDirName;
		}	

		File dir = new File(sp.getOutputDir());
		if(!dir.exists() || !dir.isDirectory()) {
			throw new RuntimeException("Output directory could not be found");
		}
		
		dir = new File(domainDir);
		if(!dir.exists() || !dir.isDirectory()) {
			throw new RuntimeException("Domain directory could not be found");
		}
		
		System.out.println("Output directory: "+sp.getOutputDir());
		System.out.println("Domain directory: "+domainDir);
	}
	
	/**
	 * Close the LP solvers
	 */
	public void close () {
		lp.close();
	}
	
	/**
	 * Solve a POMDP defined by a .POMDP file
	 * @param pomdpFileName filename of a domain in the domain directory
	 */
	public void run(String pomdpFileName) {
		// read POMDP file
		POMDP pomdp = Parser.readPOMDP(domainDir+"/"+pomdpFileName);
		
		// solve the model
		solver.solve(pomdp);
		
		// print results
		String outputFilePG = sp.getOutputDir()+"/"+pomdp.getInstanceName()+".pg";
		String outputFileAlpha = sp.getOutputDir()+"/"+pomdp.getInstanceName()+".alpha";
		System.out.println();
		System.out.println("=== RESULTS ===");
		System.out.println("Expected value: "+solver.getExpectedValue());
		System.out.println("Alpha vectors: "+outputFileAlpha);
		if(sp.dumpPolicyGraph()) System.out.println("Policy graph: "+outputFilePG);
		System.out.println("Running time: "+solver.getTotalSolveTime()+" sec");
	}	
	
	/**
	 * Main entry point of the SolvePOMDP software
	 * @param args first argument should be a filename of a .POMDP file
	 */
	public static void main(String[] args) {		
		System.out.println("SolvePOMDP v0.0.1");
		System.out.println("Author: Erwin Walraven");
		System.out.println("Web: erwinwalraven.nl/solvepomdp");
		System.out.println("Delft University of Technology");
		
		if(args.length == 0) {
			System.out.println();
			System.out.println("First argument must be the name of a file in the domains directory!");
			System.exit(0);
		}
		
		SolvePOMDP ps = new SolvePOMDP();
		ps.run(args[0]);
		ps.close();
	}
}
