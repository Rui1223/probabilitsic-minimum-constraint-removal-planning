/*The cpp file do several experiements on greedy ahnd fixedLabel search*/
/*save graphs and several corresponding solutions, respectively*/

#include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
#include "ConnectedNonOverlapGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "FixedLabelSolver.hpp"
#include "Timer.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <string> // std::string, std::to_string

int main(int argc, char** argv)
{
	int nExperiments = 1;
	int gridSize = 25;
	int nLabels = 4;
	double probPerLabel = 0.3;
	Timer t;
	std::srand(std::time(0));
	double t1;

	std::string folder_dir(argv[1]);

	for (int ii = 0; ii < nExperiments; ii++)
	{
		std::string file_dir1 = "./" + folder_dir + "/graph" + std::to_string(ii) + ".txt";
		std::string file_dir2 = "./" + folder_dir + "/FixedLabel_solution" 
															+ std::to_string(ii) + ".txt";
		std::string file_dir3 = "./" + folder_dir + "/GreedySearch_solution" 
																+ std::to_string(ii) + ".txt";

		// generate a grpah
		ConnectedGraph_t g(gridSize, gridSize, nLabels, probPerLabel);
		int start = random_generate_integer(0, gridSize*gridSize-1);
		int goal = random_generate_integer(0, gridSize*gridSize-1);
		while (start == goal)
		{
			start = random_generate_integer(0, gridSize*gridSize-1);
			goal = random_generate_integer(0, gridSize*gridSize-1);
		}
		g.write_graph(file_dir1);

		// work out on solutions
		std::cout << "----------start the fixedLabel search-------------\n";
		FixedLabelSolver_t fixedlabel_solver(g, start, goal);
		t.reset();
		fixedlabel_solver.fixedLabel_search();
		t1 = t.elapsed();
		std::cout << "\nFixedLabel time: " << t1 << " seconds\n";
		fixedlabel_solver.write_solution(file_dir2, t1);

		std::cout << "----------start the greedy search-------------\n";
		PmcrGreedySolver_t pmcr_solver(g, start, goal);
		t.reset();
		pmcr_solver.greedy_search();
		t1 = t.elapsed();
		std::cout << "\nGreedy time: " << t.elapsed() << " seconds\n";
		pmcr_solver.write_solution(file_dir3, t1);
	}

	return 0;
}



//////////////////// just generate a graph //////////////////////////
// int main(int argc, char** argv)
// {
// 	std::srand(static_cast<unsigned int>(std::time(nullptr)));
// 	Timer t;

// 	std::string folder_dir(argv[1]);
// 	std::string file_dir1 = "./" + folder_dir + "/graph" + std::to_string(0) + ".txt";

// 	int gridSize = 15;
// 	int nLabels = 5;
// 	double labelCoverage = 0.5;
// 	ConnectedNonOverlapGraph_t g(gridSize, gridSize, nLabels, labelCoverage);
// 	int start = random_generate_integer(0, gridSize*gridSize-1);
// 	int goal = random_generate_integer(0, gridSize*gridSize-1);
// 	while (start == goal)
// 	{
// 		int start = random_generate_integer(0, gridSize*gridSize-1);
// 		int goal = random_generate_integer(0, gridSize*gridSize-1);
// 	}
// 	g.write_graph(file_dir1);

// 	return 0;
// }
