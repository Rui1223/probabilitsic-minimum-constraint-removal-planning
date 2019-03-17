#include "ConnectedGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "Timer.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <string> // std::string, std::to_string
#include <vector>

int main(int argc, char** argv)
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// The file where you want to store your statistics
	std::string StatisticFolder_dir("statistics_ExecutionReplanning") // name: statistics_ExecutionReplanning

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
	double y_time;
	int y_nReplan; 
	int y_pathLength; // Manhattan distance (int)

	// Other parameters we may want to set before the experiments
	// int nProblems = 100; // number of problems we would like to work on for each single statistics (use later)
	int nProblems = 1;
	// int nGroundTruth = 100; // number of group truth (true scene) we would like to generate for EACH problem (later)
	int nGroundTruth = 2;

	// Timer to calculate the time
	Timer t_greedy;
	std::srand(std::time(0));
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

	// specify the txt file to store the statistics for Experiment 1
	std::string Exp1_stat_dir = "./" + StatisticFolder_dir + "Exp1_nObstacles/nObstacles_performance.txt";
	std::string Exp1_problem_dir = "./" + StatisticFolder_dir + "Exp1_nObstacles/Problem";
	// write into a txt file
	std::ofstream file_Exp1(Exp1_stat_dir); // an ofstream object
	// counter the problems we have worked
	int current_problem_idx;

	if (file_Exp1.is_open())
	{
		// experiment on each x_nObstacles
		for (auto const &nObs : x_Obstacles)
		{
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
			std::vector<int> posesEachObs(nObs, d_nPosesPerObs)
			// for each setting, do 100 problems
			{
				while(current_problem_idx != 100)
				{
					std::cout << "******problem: " << current_problem_idx << "for x_nObstacles: " << nObs << "******\n";
					// generate a graph problem
					ConnectedGraph_t g(d_gridSize, d_gridSize, posesEachObs);
					// save it for future visualization
					g.write_graph(Exp1_problem_dir+str(current_problem_idx)+"graphProblem");

					// solve the problem using our algorithm (multi-goal Greedy)
					// Will add other methods (A* planner, MaxSurvival planner) later for comparison
					// The things we care is just the OPTIMAL PATH
					std::cout << "*****************************************************\n";
					std::cout << "--------------start the greedy search----------------\n";
					t_greedy.reset()
					PmcrGreedySolver_t pmcr_greedy_solver(g);
					pmcr_greedy_solver.greedy_search(g);
					// The things we care about from the algorithm is just the OPTIMAL PATH
					// Before we execute the path, we want to make sure it is a high-survival path 
					// (i.e. if the survivability is 0, then there is no meaning of executing that path)
					// So let's say if the survivability is below 20% for this problem


				}
			}
		}
	}

	
	return 0;
}