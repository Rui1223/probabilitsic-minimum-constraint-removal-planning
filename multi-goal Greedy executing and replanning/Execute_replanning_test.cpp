#include "ConnectedGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "Timer.hpp"
#include "ExecuteReplanner.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <string> // std::string, std::to_string
#include <vector>

#include <bits/stdc++.h>  
#include <sys/stat.h> 
#include <sys/types.h> 

int main()
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::srand(std::time(0));
	double temp_t;

	// The file where you want to store your statistics
	std::string StatisticFolder_dir("./statistics_ExecutionReplanning"); // name: statistics_ExecutionReplanning
	mkdir(StatisticFolder_dir.c_str(), 0777);
	// Default settings
	// int d_gridSize = 50; (user later)
	int d_gridSize = 10;
	int d_nObstacles = 3;
	int d_nPosesPerObs = 3;
	// double d_distrVar = 5.0; // variance of the distribution of the obstacle

	// Parameter we play with
	// std::vector<int> x_nObstacles{5, 10, 20, 30, 40, 50}; (use later)
	std::vector<int> x_nObstacles{3};
	// std::vector<int> x_nPosesPerObs{2,3,5,7}; (use later)
	std::vector<int> x_nPosesPerObs{3};
	// std::vector<double> x_distrVar{1.0, 2.0, 3.0, 4.0, 5.0}; (use later)
	// std::vector<double> x_distrVar{3.0};

	// The measurement we care about in the experiments
	double y_time_Greedy;
	int y_nReplan_Greedy; 
	int y_pathLength_Greedy; // Manhattan distance (int)

	// Other parameters we may want to set before the experiments
	// int nProblems = 100; // number of problems we would like to work on for each single statistics (use later)
	int nProblems = 1;
	// int nGroundTruth = 100; // number of group truth (true scene) we would like to generate for EACH problem (later)
	int nGroundTruth = 1;

	// Timer to calculate the time
	Timer t_greedy;
	// Timer t_Astar;
	// Timer t_MaxSurvival;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	// Now it's time to do experiments. There could be three types of experiments
	// x_nObstacles vs (y_time, y_nReplan, y_pathLength)  ---3 figures
	// x_nPosesPerObs vs (y_time, y_nReplan, y_pathLength)  ---3 figures
	// x_distrVar vs ((y_time, y_nReplan, y_pathLength))  ---3 figures


///////////// Experiment 1: x_nObstacles vs (y_time, y_nReplan, y_pathLength)  ---3 figures ////////////////
	// For each x_nObstacles, we will do 100 problems and take the average y_time, y_nReplan &
	// y_pathLength at that certain x_Obstacles

	// create a folder under the master-folder statistics_ExecutionReplanning
	std::string nObstacles_dir = StatisticFolder_dir + "/nObstacles";
	mkdir(nObstacles_dir.c_str(), 0777);
	// specify the txt file to store the statistics for Experiment 1
	std::string Exp1_stat_txt = nObstacles_dir + "/nObstacles_performance.txt";
	// write into a txt file
	std::ofstream file_Exp1_stat(Exp1_stat_txt); // an ofstream object
	// counter the problems we are working on
	int current_problem_idx;
	// counter the ground truth we are working on
	int current_groundTruth_idx;

	// experiment on each x_nObstacles
	for (auto const &nObs : x_nObstacles)
	{
		std::string Exp1_nObs_param_dir = nObstacles_dir + "/nObstacles=" + std::to_string(nObs);
		mkdir(Exp1_nObs_param_dir.c_str(), 0777);
		y_time_Greedy = 0.0;
		y_nReplan_Greedy = 0;
		y_pathLength_Greedy = 0;
		// y_time_Astar = 0.0;
		// y_nReplan_Astar = 0;
		// y_pathLength_Astar = 0;
		// y_time_MaxSurvival = 0.0;
		// y_nReplan_MaxSurvival = 0;
		// y_pathLength_MaxSurvival = 0;

		current_problem_idx = 1;
		// specify the #poses each obstacle has. It is an input to construct a graph problem
		std::vector<int> posesEachObs(nObs, d_nPosesPerObs);
		// for each setting, do 100 problems
		while(current_problem_idx <= nProblems)
		{
			std::cout << "******problem: " << current_problem_idx << " for x_nObstacles: " << nObs << "******\n";
			// create current graph problem folder & txt
			std::string Exp1_problem_dir = Exp1_nObs_param_dir + "/problem " + std::to_string(current_problem_idx);
			mkdir(Exp1_problem_dir.c_str(), 0777);
			std::string Exp1_graph_problem_txt = Exp1_problem_dir + "/graph problem.txt";
			std::string Exp1_graph_problem_Gsolution_txt = Exp1_problem_dir + "/GreedySearch_solution.txt";
			// generate a graph problem
			ConnectedGraph_t g(d_gridSize, d_gridSize, posesEachObs);
			// save it for future visualization
			g.write_graph(Exp1_graph_problem_txt);

			// solve the problem using our algorithm (multi-goal Greedy)
			// Will add other methods (A* planner, MaxSurvival planner) later for comparison
			// The things we care is just the OPTIMAL PATH
			std::cout << "*****************************************************\n";
			std::cout << "--------------start the greedy search----------------\n";
			t_greedy.reset();
			PmcrGreedySolver_t gsolver(g);
			gsolver.greedy_search(g);
			temp_t = t_greedy.elapsed();

			// The things we care about from the algorithm is just the OPTIMAL PATH
			// Before we execute the path, we want to make sure it is a high-survival path 
			// (i.e. if the survivability is 0, then there is no meaning of executing that path)
			// So let's say if the survivability is below 70% for this problem, discard this problem
			if (gsolver.getOptimalSurvival() <= 0.7) 
			{
				std::cout << "The optimum survivability is: " << gsolver.getOptimalSurvival()
					<< ". Not a quite feasible problem to solve. Start a new problem.\n";
				continue; 
			}
			// Otherwise, this is a good problem we should work on
			// 1. record the time
			// 2. get the optimal path we are going to execute
			y_time_Greedy += temp_t;
			std::vector<int> optimalPath_Greedy = gsolver.getOptimalPath();
			gsolver.write_solution(Exp1_graph_problem_Gsolution_txt, y_time_Greedy);

			// Now we will test whether the solution from our algorithm is effective or not
			// which means we have to test it in a real scene, which is the ground truth.
			// Since the scene carries uncertainty, we will have to generate a ground truth
			// based on obstacles distribution (source of uncertainty). In order to make the scene
			// following the true distribution, we will generate 100 ground truth per problem and
			// calculate everything (time, #replan & path length) and take the average.
			// Let's do it!
			current_groundTruth_idx = 1;
			while (current_groundTruth_idx <= nGroundTruth)
			{
				std::cout << "******ground truth: " << current_groundTruth_idx << " for problem: " 
																	<< current_problem_idx << "******\n";
				// create current ground truth folder  & txt file
				std::string Exp1_groundTruth_dir = Exp1_problem_dir + "/ground truth " 
														+ std::to_string(current_groundTruth_idx);
				mkdir(Exp1_groundTruth_dir.c_str(), 0777);											
				std::string Exp1_groundTruth_txt = Exp1_groundTruth_dir + "/groundTruth.txt";

				g.generate_groundTruth(Exp1_groundTruth_txt);
				current_groundTruth_idx++;

				t_greedy.reset();
				// Execute the path on the ground truth you generate
				ExecuteReplanner_t GreedyExecutor(g, optimalPath_Greedy);
				std::vector<int> currentPath_Greedy = optimalPath_Greedy;
				while (GreedyExecutor.execute(currentPath_Greedy))
				{
					PmcrGreedySolver_t gsolver(g);
					gsolver.greedy_search(g);
					currentPath_Greedy = gsolver.getOptimalPath();

				}


			}

			current_problem_idx++;
		}
	}

	
	return 0;
}