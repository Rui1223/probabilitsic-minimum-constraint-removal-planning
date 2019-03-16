/*The cpp file do several experiements on multi-goal greedy search*/
/*save graphs and several corresponding solutions, respectively*/

#include "ConnectedGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "Timer.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <string> // std::string, std::to_string

int main(int argc, char** argv)
{
	int nExperiments = 1;
	int row = 30;
	int col = 30;
	int nObstacles = 5;
	int nPosesPerObs = 5;
	std::vector<int> nLabelsPerObs(nObstacles, nPosesPerObs);

	Timer t;
	std::srand(std::time(0));
	double t1;

	std::string folder_dir(argv[1]);

	for (int ii = 0; ii < nExperiments; ii++)
	{
		std::string file_dir1 = "./" + folder_dir + "/graph" + std::to_string(ii) + ".txt";
		std::string file_dir2 = "./" + folder_dir + "/GreedySearch_solution" 
																+ std::to_string(ii) + ".txt";

		// generate a grpah
		ConnectedGraph_t g(row, col, nLabelsPerObs);
		g.write_graph(file_dir1);
		std::cout << std::endl;

		// work out on solutions
		std::cout << "**********************************************\n";
		std::cout << "----------start the greedy search-------------\n";
		t.reset();
		PmcrGreedySolver_t pmcr_greedy_solver(g);
		pmcr_greedy_solver.greedy_search(g);
		t1 = t.elapsed();
		std::cout << "Greedy time: " << t1 << " seconds\n\n";
		pmcr_greedy_solver.write_solution(file_dir2, t1);

	}

	return 0;
}