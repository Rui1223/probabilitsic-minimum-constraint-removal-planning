#include "ConnectedGraph.hpp"
// #include "AstarSolver.hpp"
// #include "MaxSurvivalSolver.hpp"
#include "MaxLikelihoodSolver.hpp"
#include "MaxLikelihoodExecuteReplanner.hpp"
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
		// std::string file_dir2 = "./" + folder_dir + "/AstarSearch_solution" 
		// 														+ std::to_string(ii) + ".txt";
		// std::string file_dir3 = "./" + folder_dir + "/MaxSurvivalSearch_solution" 
		// 														+ std::to_string(ii) + ".txt";
		std::string file_dir4 = "./" + folder_dir + "/MaxLikelihoodSearch_solution" 
																+ std::to_string(ii) + ".txt";		

		// generate a grpah
		ConnectedGraph_t g(row, col, nLabelsPerObs, distrVar);
		g.write_graph(file_dir1);
		std::cout << std::endl;

		// work out on solutions

		std::cout << "**********************************************\n";
		std::cout << "----------start the MaxLikelihood search-------------\n";
		t.reset();
		MaxLikelihoodSolver_t mlsolver(g, g.getmStart(), g.getmGoalSet(), 
											g.getmTargetPoses(), g.getLabelWeights());
		mlsolver.MaxLikelihood_search(g);
		t1 = t.elapsed();
		std::cout << "MaxLikelihood time: " << t1 << " seconds\n\n";
		mlsolver.write_solution(file_dir4, t1);


		// generate ground truth
		std::string ground_truth_txt = "./" + folder_dir + "/ground truth.txt";
		g.generate_groundTruth(ground_truth_txt);

		MaxLikelihoodExecuteReplanner_t MLExeReplanner(g, mlsolver.getmPath());


	}

	return 0;	
}
