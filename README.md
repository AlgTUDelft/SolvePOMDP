SolvePOMDP v0.0.2   
Author: Erwin Walraven   
Web: erwinwalraven.nl/solvepomdp   
Delft University of Technology   

SolvePOMDP is a Java program that solves Partially Observable Markov Decision Processes (POMDPs). The program executes value iteration to find a POMDP solution and writes the solution to a file. The program contains both exact and approximate methods.

# Features #
* Solving POMDPs optimally using incremental pruning (Cassandra, Littman and Zhang 1997) combined with state-of-the-art vector pruning methods (Walraven and Spaan 2017).
* Computing approximate POMDP solutions using randomized point-based value iteration (Spaan and Vlassis 2005).
* POMDPs can be defined using Tony's POMDP file format, and the resulting solutions are represented by alpha vectors and policy graphs.

# Building from source #
SolvePOMDP comes with a Maven project configuration file. In order to generate an executable jar file, you only need to execute the command `mvn package`. The required libraries will be downloaded automatically from the Maven repositories. You can also import the project in your Eclipse workspace. Note that the Gurobi library file has not been included in the libs directory. This jar file can be found in your Gurobi installation directory.

# Executable binaries #
For executable binaries and a step-by-step guide we refer to the webpage of SolvePOMDP: http://erwinwalraven.nl/solvepomdp.

# References #
* Erwin Walraven and Matthijs T. J. Spaan. Accelerated Vector Pruning for Optimal POMDP Solvers. Proceedings of the 31st AAAI Conference on Artificial Intelligence, 2017.
* Matthijs T. J. Spaan and Nikos Vlassis. Perseus: Randomized Point-based Value Iteration for POMDPs. Journal of Artificial Intelligence Research, 24, pp. 195–220, 2005.
* Matthijs T. J. Spaan. Partially Observable Markov Decision Processes. Reinforcement Learning: State of the Art, pp. 387–414, Springer Verlag, 2012.
* Anthony Cassandra, Michael L. Littman and Nevin L. Zhang. Incremental Pruning: A Simple, Fast, Exact Method for Partially Observable Markov Decision Processes. Proceedings of the 13th Conference on Uncertainty in Artificial Intelligence, pp. 54–61, 1997.

