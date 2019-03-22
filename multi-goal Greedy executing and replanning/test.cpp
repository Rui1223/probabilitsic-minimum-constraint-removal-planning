#include "ConnectedGraph.hpp"
// #include "AstarSolver.hpp"
#include "MaxSurvivalSolver.hpp"
#include "Timer.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <string> // std::string, std::to_string

int main(int argc, char** argv)
{
	int nExperiments = 1;
	int row = 10;
	int col = 10;
	int nObstacles = 5;
	int nPosesPerObs = 3;
	std::vector<int> nLabelsPerObs(nObstacles, nPosesPerObs);
	double distrVar = 5.0;

	Timer t;
	std::srand(std::time(0));
	double t1;

	std::string folder_dir(argv[1]);

	for (int ii = 0; ii < nExperiments; ii++)
	{
		std::string file_dir1 = "./" + folder_dir + "/graph" + std::to_string(ii) + ".txt";
		std::string file_dir2 = "./" + folder_dir + "/AstarSearch_solution" 
																+ std::to_string(ii) + ".txt";
		std::string file_dir3 = "./" + folder_dir + "/MaxSurvivalSearch_solution" 
																+ std::to_string(ii) + ".txt";

		// generate a grpah
		ConnectedGraph_t g(row, col, nLabelsPerObs, distrVar);
		g.write_graph(file_dir1);
		std::cout << std::endl;

		// work out on solutions

		std::cout << "**********************************************\n";
		std::cout << "----------start the MaxSurvival search-------------\n";
		t.reset();
		MaxSurvivalSolver_t maxsurvival_solver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
		maxsurvival_solver.MaxSurvival_search(g);
		t1 = t.elapsed();
		std::cout << "MaxSurvival time: " << t1 << " seconds\n\n";
		maxsurvival_solver.write_solution(file_dir3, t1);

		// std::cout << "**********************************************\n";
		// std::cout << "----------start the Astar search-------------\n";
		// t.reset();
		// AstarSolver_t astar_solver(g, g.getmStart(), g.getmGoalSet(), 
		// 									g.getmTargetPoses(), g.getLabelWeights());
		// astar_solver.Astar_search(g);
		// t1 = t.elapsed();
		// std::cout << "Astar time: " << t1 << " seconds\n\n";
		// astar_solver.write_solution(file_dir2, t1);

	}

	return 0;	
}
