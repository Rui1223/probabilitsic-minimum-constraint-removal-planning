#include "ConnectedGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "AstarSolver.hpp"
#include "MaxSurvivalSolver.hpp"
#include "MaxLikelihoodSolver.hpp"
#include "Timer.hpp"
#include "GreedyExecuteReplanner.hpp"
#include "AstarExecuteReplanner.hpp"
#include "MaxSurvivalExecuteReplanner.hpp"
#include "MaxLikelihoodExecuteReplanner.hpp"

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
	int d_gridSize = 50; // will use later for whole experiment
	// int d_gridSize = 50;
	int d_nObstacles = 30; // will use later for whole experiment
	// int d_nObstacles = 30;
	int d_nPosesPerObs = 5; // will use later for whole experiment
	// int d_nPosesPerObs = 5;
	double d_distrVar = 3.0; // variance of the distribution of the obstacle (use later)
	// double d_distrVar = 3.0;

	// Parameter we play with
	std::vector<int> x_nObstacles{10, 20, 30, 40, 50, 60}; // use later
	// std::vector<int> x_nObstacles{30};
	std::vector<int> x_nPosesPerObs{2, 3, 5, 7, 8 ,9}; // use later
	// std::vector<int> x_nPosesPerObs{5};
	std::vector<double> x_distrVar{0.0, 1.0, 3.0, 5.0, 10.0, 100.0}; // use later 
	// std::vector<double> x_distrVar{3.0};

	// Other parameters we may want to set before the experiments
	// number of problems we would like to work on for each single statistics
	int nProblems = 100;
	// number of group truth (true scene) we would like to generate for EACH problem
	int nGroundTruth = 100;
	// counter the problems we are working on
	int current_problem_idx;
	// counter the ground truth we are working on
	int current_groundTruth_idx;

	// Timer to calculate the time
	Timer t_greedy;
	Timer t_Astar;
	Timer t_MaxSurvival;
	Timer t_MaxLikelihood;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	// Now it's time to do experiments. There could be three types of experiments
	// x_nObstacles vs (y_time, y_nReplan, y_pathLength)  ---3 figures
	// x_nPosesPerObs vs (y_time, y_nReplan, y_pathLength)  ---3 figures
	// x_distrVar vs ((y_time, y_nReplan, y_pathLength))  ---3 figures


///////////// Experiment 1: x_nObstacles vs (y_time, y_nReplan, y_pathLength)  ---3 figures ////////////////
	// For each x_nObstacles, we will do 100 problems and take the average y_time, y_nReplan &
	// y_pathLength at that certain x_nObstacles

	// create a folder under the master-folder statistics_ExecutionReplanning
	std::string nObstacles_dir = StatisticFolder_dir + "/nObstacles";
	mkdir(nObstacles_dir.c_str(), 0777);
	// specify the txt file to store the statistics for Experiment 1
	std::string Exp1_stat_txt = nObstacles_dir + "/nObstacles_performance.txt";
	// write into a txt file
	std::ofstream file_Exp1_stat(Exp1_stat_txt); // an ofstream object

	// experiment on each x_nObstacles
	for (auto const &nObs : x_nObstacles)
	{
		std::string Exp1_nObs_param_dir = nObstacles_dir + "/nObstacles=" + std::to_string(nObs);
		mkdir(Exp1_nObs_param_dir.c_str(), 0777);
		// The measurement we care about in the experiments
		double y_time_Greedy = 0.0;
		int y_nReplan_Greedy = 0; 
		int y_pathLength_Greedy = 0; // Manhattan distance (int)
		double y_pathUtilityRate_Greedy = 0.0;
		double y_time_Astar = 0.0;
		int y_nReplan_Astar = 0;
		int y_pathLength_Astar = 0;
		double y_pathUtilityRate_Astar = 0.0;
		double y_time_MaxSurvival = 0.0;
		int y_nReplan_MaxSurvival = 0;
		int y_pathLength_MaxSurvival = 0;
		double y_pathUtilityRate_MaxSurvival = 0.0;
		double y_time_MaxLikelihood = 0.0;
		int y_nReplan_MaxLikelihood = 0;
		int y_pathLength_MaxLikelihood = 0;
		double y_pathUtilityRate_MaxLikelihood = 0.0;

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
			std::string Exp1_graph_problem_Asolution_txt = Exp1_problem_dir + "/AstarSearch_solution.txt";
			std::string Exp1_graph_problem_Msolution_txt = Exp1_problem_dir + "/MaxSurvivalSearch_solution.txt";
			std::string Exp1_graph_problem_MLsolution_txt = Exp1_problem_dir + "/MaxLikelihoodSearch_solution.txt";
			// generate a graph problem
			ConnectedGraph_t g(d_gridSize, d_gridSize, posesEachObs, d_distrVar);
			// save it for future visualization
			g.write_graph(Exp1_graph_problem_txt);

			// solve the problem using our algorithm (multi-goal Greedy, A* planner, MaxSurvival planner)
			// The things we care is just the OPTIMAL PATH
			std::cout << "\n--------------start the MaximumSuccess Greedy Search----------------\n";
			t_greedy.reset();
			PmcrGreedySolver_t gsolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
			gsolver.greedy_search(g);
			temp_t = t_greedy.elapsed();
			// The things we care about from the algorithm is just the OPTIMAL PATH
			// Before we proceed, let's make sure the graph problem is quite solvable
			if (gsolver.getOptimalSurvival() <= 0.4 or gsolver.getIsSolvable() == false) 
			{
				std::cout << "The optimum survivability is: " << gsolver.getOptimalSurvival()
					<< ". Not a quite feasible problem to solve. Start a new problem.\n";
				continue; 
			}
			// Otherwise, this is a good problem we should work on
			// 1. record the time and write in the solution
			// 2. Run other naive methods
			// 2. get the optimal path we are going to execute
			y_time_Greedy += temp_t;
			gsolver.write_solution(Exp1_graph_problem_Gsolution_txt, y_time_Greedy);

			std::cout << "\n--------------start the Astar Search----------------\n";
			t_Astar.reset();
			AstarSolver_t asolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
			asolver.Astar_search(g);
			temp_t = t_Astar.elapsed();			
			y_time_Astar += temp_t;
			asolver.write_solution(Exp1_graph_problem_Asolution_txt, y_time_Astar);

			std::cout << "\n--------------start the MaxSurvival Search----------------\n";
			t_MaxSurvival.reset();
			MaxSurvivalSolver_t msolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
			msolver.MaxSurvival_search(g);
			temp_t = t_MaxSurvival.elapsed();			
			y_time_MaxSurvival += temp_t;
			msolver.write_solution(Exp1_graph_problem_Msolution_txt, y_time_MaxSurvival);


			std::cout << "\n--------------start the MaxLikelihood Search----------------\n";
			t_MaxLikelihood.reset();
			MaxLikelihoodSolver_t mlsolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
			mlsolver.MaxLikelihood_search(g);
			temp_t = t_MaxLikelihood.elapsed();			
			y_time_MaxLikelihood += temp_t;
			mlsolver.write_solution(Exp1_graph_problem_MLsolution_txt, y_time_MaxLikelihood);

			// Now we will test whether the solution from our algorithms are effective or not
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

				// generate ground truth
				g.generate_groundTruth(Exp1_groundTruth_txt);
				// Execute the path each planner computes on the ground truth you generate
				GreedyExecuteReplanner_t GExeReplanner(g, gsolver.getOptimalPath());
				// At this point you either
				// (1) finish execution & replan & find the goal
				// (2) the ground truth is doomed
				if (GExeReplanner.getIsDoomed()) {continue;}
				else
				{
					y_time_Greedy += GExeReplanner.getExecutionTime();
					y_nReplan_Greedy += GExeReplanner.getnReplan();
					y_pathLength_Greedy += GExeReplanner.getPathLength();
					y_pathUtilityRate_Greedy += GExeReplanner.getPathUtilityRate();
				}
				// You are reaching here since the ground truth is not doomed
				// Call replanner for other naive methods
				AstarExecuteReplanner_t AExeReplanner(g, asolver.getmPath());
				y_time_Astar += AExeReplanner.getExecutionTime();
				y_nReplan_Astar += AExeReplanner.getnReplan();
				y_pathLength_Astar += AExeReplanner.getPathLength();
				y_pathUtilityRate_Astar += AExeReplanner.getPathUtilityRate();

				MaxSurvivalExecuteReplanner_t MExeReplanner(g, msolver.getmPath());
				y_time_MaxSurvival += MExeReplanner.getExecutionTime();
				y_nReplan_MaxSurvival += MExeReplanner.getnReplan();
				y_pathLength_MaxSurvival += MExeReplanner.getPathLength();
				y_pathUtilityRate_MaxSurvival += MExeReplanner.getPathUtilityRate();

				MaxLikelihoodExecuteReplanner_t MLExeReplanner(g, mlsolver.getmPath());
				y_time_MaxLikelihood += MLExeReplanner.getExecutionTime();
				y_nReplan_MaxLikelihood += MLExeReplanner.getnReplan();
				y_pathLength_MaxLikelihood += MLExeReplanner.getPathLength();
				y_pathUtilityRate_MaxLikelihood += MLExeReplanner.getPathUtilityRate();

				current_groundTruth_idx++;
				std::cout << "Start creating a new ground truth\n";

			}

			current_problem_idx++;
			std::cout << "start creating a new problem\n";
		}
		std::cout << "Finish all problems for the current parameter. "
					<< "Now let's write in the performance: x_nObstacles=" << nObs << "\n";
		double y_average_time_G = y_time_Greedy*1.0 / nProblems / nGroundTruth;
		double y_average_nReplan_G = y_nReplan_Greedy*1.0 / nProblems / nGroundTruth;
		double y_average_pathLength_G = y_pathLength_Greedy*1.0 / nProblems / nGroundTruth;
		double y_average_pathUtilityRate_G = y_pathUtilityRate_Greedy*1.0 / nProblems / nGroundTruth;
		double y_average_time_A = y_time_Astar*1.0 / nProblems / nGroundTruth;
		double y_average_nReplan_A = y_nReplan_Astar*1.0 / nProblems / nGroundTruth;
		double y_average_pathLength_A = y_pathLength_Astar*1.0 / nProblems / nGroundTruth;
		double y_average_pathUtilityRate_A = y_pathUtilityRate_Astar*1.0 / nProblems / nGroundTruth;
		double y_average_time_M = y_time_MaxSurvival*1.0 / nProblems / nGroundTruth;
		double y_average_nReplan_M = y_nReplan_MaxSurvival*1.0 / nProblems / nGroundTruth;
		double y_average_pathLength_M = y_pathLength_MaxSurvival*1.0 / nProblems / nGroundTruth;
		double y_average_pathUtilityRate_M = y_pathUtilityRate_MaxSurvival*1.0 / nProblems / nGroundTruth;
		double y_average_time_ML = y_time_MaxLikelihood*1.0 / nProblems / nGroundTruth;
		double y_average_nReplan_ML = y_nReplan_MaxLikelihood*1.0 / nProblems / nGroundTruth;
		double y_average_pathLength_ML = y_pathLength_MaxLikelihood*1.0 / nProblems / nGroundTruth;
		double y_average_pathUtilityRate_ML = y_pathUtilityRate_MaxLikelihood*1.0 / nProblems / nGroundTruth;


		if (file_Exp1_stat.is_open())
		{
			std::cout << "start writing the statistics\n";
			file_Exp1_stat << nObs << " " << y_average_time_G << " " << y_average_nReplan_G << " "
				<< y_average_pathLength_G <<  " " << y_average_pathUtilityRate_G << " " << y_average_time_A 
				<< " " << y_average_nReplan_A << " " << y_average_pathLength_A << " " << y_average_pathUtilityRate_A
				<< " " << y_average_time_M << " " << y_average_nReplan_M << " " << y_average_pathLength_M 
				<< " " << y_average_pathUtilityRate_M << " " << y_average_time_ML << " " <<  y_average_nReplan_ML
				<< " " << y_average_pathLength_ML << " " << y_average_pathUtilityRate_ML << "\n";
		}
		std::cout << "Finish writing for the current parameter x_nObstacles=" << nObs << "\n";
		std::cout << "start a new parameter\n";

	}
	std::cout << "Finish all the parameters\n";
	file_Exp1_stat.close();


///////////// Experiment 2: x_nPosesPerObs vs (y_time, y_nReplan, y_pathLength)  ---3 figures ////////////////
	// For each x_nPosesPerObs, we will do 100 problems and take the average y_time, y_nReplan &
	// y_pathLength at that certain x_nPosesPerObs

	// create a folder under the master-folder statistics_ExecutionReplanning
	std::string nPosesPerObs_dir = StatisticFolder_dir + "/nPosesPerObs";
	mkdir(nPosesPerObs_dir.c_str(), 0777);
	// specify the txt file to store the statistics for Experiment 2
	std::string Exp2_stat_txt = nPosesPerObs_dir + "/nPosesPerObs_performance.txt";
	// write into a txt file
	std::ofstream file_Exp2_stat(Exp2_stat_txt); // an ofstream object

	// experiment on each x_nPosesPerObs
	for (auto const &nPPO : x_nPosesPerObs)
	{
		std::string Exp2_nPPO_param_dir = nPosesPerObs_dir + "/nPosesPerObs=" + std::to_string(nPPO);
		mkdir(Exp2_nPPO_param_dir.c_str(), 0777);
		// The measurement we care about in the experiments
		double y_time_Greedy = 0.0;
		int y_nReplan_Greedy = 0; 
		int y_pathLength_Greedy = 0; // Manhattan distance (int)
		double y_pathUtilityRate_Greedy = 0.0;
		double y_time_Astar = 0.0;
		int y_nReplan_Astar = 0;
		int y_pathLength_Astar = 0;
		double y_pathUtilityRate_Astar = 0.0;
		double y_time_MaxSurvival = 0.0;
		int y_nReplan_MaxSurvival = 0;
		int y_pathLength_MaxSurvival = 0;
		double y_pathUtilityRate_MaxSurvival = 0.0;
		double y_time_MaxLikelihood = 0.0;
		int y_nReplan_MaxLikelihood = 0;
		int y_pathLength_MaxLikelihood = 0;
		double y_pathUtilityRate_MaxLikelihood = 0.0;

		current_problem_idx = 1;
		// specify the #poses each obstacle has. It is an input to construct a graph problem
		std::vector<int> posesEachObs(d_nObstacles, nPPO);
		// for each setting, do 100 problems
		while(current_problem_idx <= nProblems)
		{
			std::cout << "******problem: " << current_problem_idx << " for x_nPosesPerObs: " << nPPO << "******\n";
			// create current graph problem folder & txt
			std::string Exp2_problem_dir = Exp2_nPPO_param_dir + "/problem " + std::to_string(current_problem_idx);
			mkdir(Exp2_problem_dir.c_str(), 0777);
			std::string Exp2_graph_problem_txt = Exp2_problem_dir + "/graph problem.txt";
			std::string Exp2_graph_problem_Gsolution_txt = Exp2_problem_dir + "/GreedySearch_solution.txt";
			std::string Exp2_graph_problem_Asolution_txt = Exp2_problem_dir + "/AstarSearch_solution.txt";
			std::string Exp2_graph_problem_Msolution_txt = Exp2_problem_dir + "/MaxSurvivalSearch_solution.txt";
			std::string Exp2_graph_problem_MLsolution_txt = Exp2_problem_dir + "/MaxLikelihoodSearch_solution.txt";
			// generate a graph problem
			ConnectedGraph_t g(d_gridSize, d_gridSize, posesEachObs, d_distrVar);
			// save it for future visualization
			g.write_graph(Exp2_graph_problem_txt);

			// solve the problem using our algorithm (multi-goal Greedy, A* planner, MaxSurvival planner)
			// The things we care is just the OPTIMAL PATH
			std::cout << "\n--------------start the MaximumSuccess Greedy Search----------------\n";
			t_greedy.reset();
			PmcrGreedySolver_t gsolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
			gsolver.greedy_search(g);
			temp_t = t_greedy.elapsed();
			// The things we care about from the algorithm is just the OPTIMAL PATH
			// Before we proceed, let's make sure the graph problem is quite solvable
			if (gsolver.getOptimalSurvival() <= 0.4 or gsolver.getIsSolvable() == false) 
			{
				std::cout << "The optimum survivability is: " << gsolver.getOptimalSurvival()
					<< ". Not a quite feasible problem to solve. Start a new problem.\n";
				continue; 
			}
			// Otherwise, this is a good problem we should work on
			// 1. record the time and write in the solution
			// 2. Run other naive methods
			// 2. get the optimal path we are going to execute
			y_time_Greedy += temp_t;
			gsolver.write_solution(Exp2_graph_problem_Gsolution_txt, y_time_Greedy);

			std::cout << "\n--------------start the Astar Search----------------\n";
			t_Astar.reset();
			AstarSolver_t asolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
			asolver.Astar_search(g);
			temp_t = t_Astar.elapsed();			
			y_time_Astar += temp_t;
			asolver.write_solution(Exp2_graph_problem_Asolution_txt, y_time_Astar);

			std::cout << "\n--------------start the MaxSurvival Search----------------\n";
			t_MaxSurvival.reset();
			MaxSurvivalSolver_t msolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
			msolver.MaxSurvival_search(g);
			temp_t = t_MaxSurvival.elapsed();			
			y_time_MaxSurvival += temp_t;
			msolver.write_solution(Exp2_graph_problem_Msolution_txt, y_time_MaxSurvival);

			std::cout << "\n--------------start the MaxLikelihood Search----------------\n";
			t_MaxLikelihood.reset();
			MaxLikelihoodSolver_t mlsolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
			mlsolver.MaxLikelihood_search(g);
			temp_t = t_MaxLikelihood.elapsed();			
			y_time_MaxLikelihood += temp_t;
			mlsolver.write_solution(Exp2_graph_problem_MLsolution_txt, y_time_MaxLikelihood);

			// Now we will test whether the solution from our algorithms are effective or not
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
				std::string Exp2_groundTruth_dir = Exp2_problem_dir + "/ground truth " 
														+ std::to_string(current_groundTruth_idx);
				mkdir(Exp2_groundTruth_dir.c_str(), 0777);											
				std::string Exp2_groundTruth_txt = Exp2_groundTruth_dir + "/groundTruth.txt";

				// generate ground truth
				g.generate_groundTruth(Exp2_groundTruth_txt);
				// Execute the path on the ground truth you generate
				GreedyExecuteReplanner_t GExeReplanner(g, gsolver.getOptimalPath());
				// At this point you either
				// (1) finish execution & replan & find the goal
				// (2) the ground truth is doomed
				if (GExeReplanner.getIsDoomed()) {continue;}
				else
				{
					y_time_Greedy += GExeReplanner.getExecutionTime();
					y_nReplan_Greedy += GExeReplanner.getnReplan();
					y_pathLength_Greedy += GExeReplanner.getPathLength();
					y_pathUtilityRate_Greedy += GExeReplanner.getPathUtilityRate();
				}
				// You are reaching here since the ground truth is not doomed
				// Call replanner for other naive methods
				AstarExecuteReplanner_t AExeReplanner(g, asolver.getmPath());
				y_time_Astar += AExeReplanner.getExecutionTime();
				y_nReplan_Astar += AExeReplanner.getnReplan();
				y_pathLength_Astar += AExeReplanner.getPathLength();
				y_pathUtilityRate_Astar += AExeReplanner.getPathUtilityRate();

				MaxSurvivalExecuteReplanner_t MExeReplanner(g, msolver.getmPath());
				y_time_MaxSurvival += MExeReplanner.getExecutionTime();
				y_nReplan_MaxSurvival += MExeReplanner.getnReplan();
				y_pathLength_MaxSurvival += MExeReplanner.getPathLength();
				y_pathUtilityRate_MaxSurvival += MExeReplanner.getPathUtilityRate();

				MaxLikelihoodExecuteReplanner_t MLExeReplanner(g, mlsolver.getmPath());
				y_time_MaxLikelihood += MLExeReplanner.getExecutionTime();
				y_nReplan_MaxLikelihood += MLExeReplanner.getnReplan();
				y_pathLength_MaxLikelihood += MLExeReplanner.getPathLength();
				y_pathUtilityRate_MaxLikelihood += MLExeReplanner.getPathUtilityRate();
				
				current_groundTruth_idx++;
				std::cout << "Start creating a new ground truth\n";				
			}
			current_problem_idx++;
			std::cout << "start creating a new problem\n";
		}
		std::cout << "Finish all problems for current parameter. "
						<< "Now let's write in the performance: x_nPosesPerObs=" << nPPO << "\n";
		double y_average_time_G = y_time_Greedy*1.0 / nProblems / nGroundTruth;
		double y_average_nReplan_G = y_nReplan_Greedy*1.0 / nProblems / nGroundTruth;
		double y_average_pathLength_G = y_pathLength_Greedy*1.0 / nProblems / nGroundTruth;
		double y_average_pathUtilityRate_G = y_pathUtilityRate_Greedy*1.0 / nProblems / nGroundTruth;
		double y_average_time_A = y_time_Astar*1.0 / nProblems / nGroundTruth;
		double y_average_nReplan_A = y_nReplan_Astar*1.0 / nProblems / nGroundTruth;
		double y_average_pathLength_A = y_pathLength_Astar*1.0 / nProblems / nGroundTruth;
		double y_average_pathUtilityRate_A = y_pathUtilityRate_Astar*1.0 / nProblems / nGroundTruth;
		double y_average_time_M = y_time_MaxSurvival*1.0 / nProblems / nGroundTruth;
		double y_average_nReplan_M = y_nReplan_MaxSurvival*1.0 / nProblems / nGroundTruth;
		double y_average_pathLength_M = y_pathLength_MaxSurvival*1.0 / nProblems / nGroundTruth;
		double y_average_pathUtilityRate_M = y_pathUtilityRate_MaxSurvival*1.0 / nProblems / nGroundTruth;
		double y_average_time_ML = y_time_MaxLikelihood*1.0 / nProblems / nGroundTruth;
		double y_average_nReplan_ML = y_nReplan_MaxLikelihood*1.0 / nProblems / nGroundTruth;
		double y_average_pathLength_ML = y_pathLength_MaxLikelihood*1.0 / nProblems / nGroundTruth;
		double y_average_pathUtilityRate_ML = y_pathUtilityRate_MaxLikelihood*1.0 / nProblems / nGroundTruth;

		if (file_Exp2_stat.is_open())
		{
			std::cout << "start writing the statistics\n";
			file_Exp2_stat << nPPO << " " << y_average_time_G << " " << y_average_nReplan_G << " "
				<< y_average_pathLength_G <<  " " << y_average_pathUtilityRate_G << " " << y_average_time_A 
				<< " " << y_average_nReplan_A << " " << y_average_pathLength_A << " " << y_average_pathUtilityRate_A
				<< " " << y_average_time_M << " " << y_average_nReplan_M << " " << y_average_pathLength_M 
				<< " " << y_average_pathUtilityRate_M << " " << y_average_time_ML << " " <<  y_average_nReplan_ML
				<< " " << y_average_pathLength_ML << " " << y_average_pathUtilityRate_ML << "\n";
		}
		std::cout << "Finish writing for the current parameter x_nPosesPerObs=" << nPPO << "\n";
		std::cout << "start a new parameter\n";

	}
	std::cout << "Finish all the parameters\n";
	file_Exp2_stat.close();


///////////// Experiment 3: x_distrVar vs (y_time, y_nReplan, y_pathLength)  ---3 figures ////////////////
	// For each x_distrVar, we will do 100 problems and take the average y_time, y_nReplan &
	// y_pathLength at that certain x_distrVar

	// create a folder under the master-folder statistics_ExecutionReplanning
	std::string distrVar_dir = StatisticFolder_dir + "/distrVar";
	mkdir(distrVar_dir.c_str(), 0777);
	// specify the txt file to store the statistics for Experiment 3
	std::string Exp3_stat_txt = distrVar_dir + "/distrVar_performance.txt";
	// write into a txt file
	std::ofstream file_Exp3_stat(Exp3_stat_txt); // an ofstream object

	// experiment on each x_nPosesPerObs
	for (auto const &distrV : x_distrVar)
	{
		std::string Exp3_distrV_param_dir = distrVar_dir + "/distrVar=" + std::to_string(distrV);
		mkdir(Exp3_distrV_param_dir.c_str(), 0777);
		// The measurement we care about in the experiments
		double y_time_Greedy = 0.0;
		int y_nReplan_Greedy = 0; 
		int y_pathLength_Greedy = 0; // Manhattan distance (int)
		double y_pathUtilityRate_Greedy = 0.0;
		double y_time_Astar = 0.0;
		int y_nReplan_Astar = 0;
		int y_pathLength_Astar = 0;
		double y_pathUtilityRate_Astar = 0.0;
		double y_time_MaxSurvival = 0.0;
		int y_nReplan_MaxSurvival = 0;
		int y_pathLength_MaxSurvival = 0;
		double y_pathUtilityRate_MaxSurvival = 0.0;
		double y_time_MaxLikelihood = 0.0;
		int y_nReplan_MaxLikelihood = 0;
		int y_pathLength_MaxLikelihood = 0;
		double y_pathUtilityRate_MaxLikelihood = 0.0;

		double currentEntropy = 0.0;

		current_problem_idx = 1;
		// specify the #poses each obstacle has. It is an input to construct a graph problem
		std::vector<int> posesEachObs(d_nObstacles, d_nPosesPerObs);
		// for each setting, do 100 problems
		while(current_problem_idx <= nProblems)
		{
			std::cout << "******problem: " << current_problem_idx << " for x_distrVar: " << distrV << "******\n";
			// create current graph problem folder & txt
			std::string Exp3_problem_dir = Exp3_distrV_param_dir + "/problem " + std::to_string(current_problem_idx);
			mkdir(Exp3_problem_dir.c_str(), 0777);
			std::string Exp3_graph_problem_txt = Exp3_problem_dir + "/graph problem.txt";
			std::string Exp3_graph_problem_Gsolution_txt = Exp3_problem_dir + "/GreedySearch_solution.txt";
			std::string Exp3_graph_problem_Asolution_txt = Exp3_problem_dir + "/AstarSearch_solution.txt";
			std::string Exp3_graph_problem_Msolution_txt = Exp3_problem_dir + "/MaxSurvivalSearch_solution.txt";
			std::string Exp3_graph_problem_MLsolution_txt = Exp3_problem_dir + "/MaxLikelihoodSearch_solution.txt";
			// generate a graph problem
			ConnectedGraph_t g(d_gridSize, d_gridSize, posesEachObs, distrV);
			// save it for future visualization
			g.write_graph(Exp3_graph_problem_txt);
			currentEntropy += g.getEntropy();

			// solve the problem using our algorithm (multi-goal Greedy, A* planner, MaxSurvival planner)
			// The things we care is just the OPTIMAL PATH
			std::cout << "\n--------------start the MaximumSuccess Greedy Search----------------\n";
			t_greedy.reset();
			PmcrGreedySolver_t gsolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
			gsolver.greedy_search(g);
			temp_t = t_greedy.elapsed();
			// The things we care about from the algorithm is just the OPTIMAL PATH
			// Before we proceed, let's make sure the graph problem is quite solvable
			if (gsolver.getOptimalSurvival() <= 0.4 or gsolver.getIsSolvable() == false) 
			{
				std::cout << "The optimum survivability is: " << gsolver.getOptimalSurvival()
					<< ". Not a quite feasible problem to solve. Start a new problem.\n";
				continue; 
			}
			// Otherwise, this is a good problem we should work on
			// 1. record the time and write in the solution
			// 2. Run other naive methods
			// 2. get the optimal path we are going to execute
			y_time_Greedy += temp_t;
			gsolver.write_solution(Exp3_graph_problem_Gsolution_txt, y_time_Greedy);

			std::cout << "\n--------------start the Astar Search----------------\n";
			t_Astar.reset();
			AstarSolver_t asolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
			asolver.Astar_search(g);
			temp_t = t_Astar.elapsed();			
			y_time_Astar += temp_t;
			asolver.write_solution(Exp3_graph_problem_Asolution_txt, y_time_Astar);

			std::cout << "\n--------------start the MaxSurvival Search----------------\n";
			t_MaxSurvival.reset();
			MaxSurvivalSolver_t msolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
			msolver.MaxSurvival_search(g);
			temp_t = t_MaxSurvival.elapsed();			
			y_time_MaxSurvival += temp_t;
			msolver.write_solution(Exp3_graph_problem_Msolution_txt, y_time_MaxSurvival);

			std::cout << "\n--------------start the MaxLikelihood Search----------------\n";
			t_MaxLikelihood.reset();
			MaxLikelihoodSolver_t mlsolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
			mlsolver.MaxLikelihood_search(g);
			temp_t = t_MaxLikelihood.elapsed();			
			y_time_MaxLikelihood += temp_t;
			mlsolver.write_solution(Exp3_graph_problem_MLsolution_txt, y_time_MaxLikelihood);	

			// Now we will test whether the solution from our algorithms are effective or not
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
				std::string Exp3_groundTruth_dir = Exp3_problem_dir + "/ground truth " 
														+ std::to_string(current_groundTruth_idx);
				mkdir(Exp3_groundTruth_dir.c_str(), 0777);											
				std::string Exp3_groundTruth_txt = Exp3_groundTruth_dir + "/groundTruth.txt";

				// generate ground truth
				g.generate_groundTruth(Exp3_groundTruth_txt);
				// Execute the path on the ground truth you generate
				GreedyExecuteReplanner_t GExeReplanner(g, gsolver.getOptimalPath());
				// At this point you either
				// (1) finish execution & replan & find the goal
				// (2) the ground truth is doomed
				if (GExeReplanner.getIsDoomed()) {continue;}
				else
				{
					y_time_Greedy += GExeReplanner.getExecutionTime();
					y_nReplan_Greedy += GExeReplanner.getnReplan();
					y_pathLength_Greedy += GExeReplanner.getPathLength();
					y_pathUtilityRate_Greedy += GExeReplanner.getPathUtilityRate();
				}
				// You are reaching here since the ground truth is not doomed
				// Call replanner for other naive methods
				AstarExecuteReplanner_t AExeReplanner(g, asolver.getmPath());
				y_time_Astar += AExeReplanner.getExecutionTime();
				y_nReplan_Astar += AExeReplanner.getnReplan();
				y_pathLength_Astar += AExeReplanner.getPathLength();
				y_pathUtilityRate_Astar += AExeReplanner.getPathUtilityRate();

				MaxSurvivalExecuteReplanner_t MExeReplanner(g, msolver.getmPath());
				y_time_MaxSurvival += MExeReplanner.getExecutionTime();
				y_nReplan_MaxSurvival += MExeReplanner.getnReplan();
				y_pathLength_MaxSurvival += MExeReplanner.getPathLength();
				y_pathUtilityRate_MaxSurvival += MExeReplanner.getPathUtilityRate();

				MaxLikelihoodExecuteReplanner_t MLExeReplanner(g, mlsolver.getmPath());
				y_time_MaxLikelihood += MLExeReplanner.getExecutionTime();
				y_nReplan_MaxLikelihood += MLExeReplanner.getnReplan();
				y_pathLength_MaxLikelihood += MLExeReplanner.getPathLength();
				y_pathUtilityRate_MaxLikelihood += MLExeReplanner.getPathUtilityRate();

				current_groundTruth_idx++;
				std::cout << "Start creating a new ground truth\n";				
			}
			current_problem_idx++;
			std::cout << "start creating a new problem\n";
		}
		std::cout << "Finish all problems for current parameter. "
						<< "Now let's write in the performance: x_distrVar=" << distrV << "\n";
		double y_average_time_G = y_time_Greedy*1.0 / nProblems / nGroundTruth;
		double y_average_nReplan_G = y_nReplan_Greedy*1.0 / nProblems / nGroundTruth;
		double y_average_pathLength_G = y_pathLength_Greedy*1.0 / nProblems / nGroundTruth;
		double y_average_pathUtilityRate_G = y_pathUtilityRate_Greedy*1.0 / nProblems / nGroundTruth;
		double y_average_time_A = y_time_Astar*1.0 / nProblems / nGroundTruth;
		double y_average_nReplan_A = y_nReplan_Astar*1.0 / nProblems / nGroundTruth;
		double y_average_pathLength_A = y_pathLength_Astar*1.0 / nProblems / nGroundTruth;
		double y_average_pathUtilityRate_A = y_pathUtilityRate_Astar*1.0 / nProblems / nGroundTruth;
		double y_average_time_M = y_time_MaxSurvival*1.0 / nProblems / nGroundTruth;
		double y_average_nReplan_M = y_nReplan_MaxSurvival*1.0 / nProblems / nGroundTruth;
		double y_average_pathLength_M = y_pathLength_MaxSurvival*1.0 / nProblems / nGroundTruth;
		double y_average_pathUtilityRate_M = y_pathUtilityRate_MaxSurvival*1.0 / nProblems / nGroundTruth;
		double y_average_time_ML = y_time_MaxLikelihood*1.0 / nProblems / nGroundTruth;
		double y_average_nReplan_ML = y_nReplan_MaxLikelihood*1.0 / nProblems / nGroundTruth;
		double y_average_pathLength_ML = y_pathLength_MaxLikelihood*1.0 / nProblems / nGroundTruth;
		double y_average_pathUtilityRate_ML = y_pathUtilityRate_MaxLikelihood*1.0 / nProblems / nGroundTruth;
		currentEntropy = currentEntropy / nProblems;

		if (file_Exp3_stat.is_open())
		{
			std::cout << "start writing the statistics\n";
			file_Exp3_stat << currentEntropy << " " << y_average_time_G << " " << y_average_nReplan_G << " "
				<< y_average_pathLength_G <<  " " << y_average_pathUtilityRate_G << " " << y_average_time_A 
				<< " " << y_average_nReplan_A << " " << y_average_pathLength_A << " " << y_average_pathUtilityRate_A
				<< " " << y_average_time_M << " " << y_average_nReplan_M << " " << y_average_pathLength_M 
				<< " " << y_average_pathUtilityRate_M << " " << y_average_time_ML << " " <<  y_average_nReplan_ML
				<< " " << y_average_pathLength_ML << " " << y_average_pathUtilityRate_ML << "\n";
		}
		std::cout << "Finish writing for the current parameter x_distrVar=" << distrV << "\n";
		std::cout << "start a new parameter\n";

	}
	std::cout << "Finish all the parameters\n";
	file_Exp3_stat.close();

	
	return 0;
}