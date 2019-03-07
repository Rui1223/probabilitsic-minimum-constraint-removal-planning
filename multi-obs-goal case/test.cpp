/*The cpp file do several experiements on greedy ahnd fixedLabel search*/
/*save graphs and several corresponding solutions, respectively*/

// #include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
// #include "ConnectedNonOverlapGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "FixedLabelSolver.hpp"
// #include "GrowingTreeSolver.hpp"
#include "Timer.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <string> // std::string, std::to_string

int main(int argc, char** argv)
{
	int nExperiments = 1;
	int row = 20;
	int col = 30;
	std::vector<int> nLabelsPerObs{4,4,4,4};
	double probPerLabel = 0.6;
	Timer t;
	std::srand(std::time(0));
	double t1;
	int start;
	int goal;
	std::vector<int> start_neighbors;

	std::string folder_dir(argv[1]);

	for (int ii = 0; ii < nExperiments; ii++)
	{
		std::string file_dir1 = "./" + folder_dir + "/graph" + std::to_string(ii) + ".txt";
		// std::string file_dir2 = "./" + folder_dir + "/FixedLabel_solution" 
		// 													+ std::to_string(ii) + ".txt";
		std::string file_dir3 = "./" + folder_dir + "/GreedySearch_solution" 
																+ std::to_string(ii) + ".txt";		
		// std::string file_dir4 = "./" + folder_dir + "/GrowingTree_solution" 
		// 														+ std::to_string(ii) + ".txt";

		// generate a grpah
		ConnectedGraph_t g(row, col, nLabelsPerObs, probPerLabel);

		g.write_graph(file_dir1);
		std::cout << std::endl;

		// work out on solutions
		// std::cout << "----------start the fixedLabel search-------------\n";
		// FixedLabelSolver_t fixedlabel_solver(g, start, goal);
		// t.reset();
		// std::cout << "Start to compute time\n";
		// fixedlabel_solver.fixedLabel_search();
		// t1 = t.elapsed();
		// std::cout << "FixedLabel time: " << t1 << " seconds\n\n";
		// fixedlabel_solver.write_solution(file_dir2, t1);

		std::cout << "----------start the greedy search-------------\n";
		PmcrGreedySolver_t pmcr_greedy_solver(g);
		t.reset();
		std::cout << "Start to compute time\n";
		pmcr_greedy_solver.greedy_search();
		t1 = t.elapsed();
		std::cout << "Greedy time: " << t1 << " seconds\n\n";
		pmcr_greedy_solver.write_solution(file_dir3, t1);

		// std::cout << "----------start the growingtree search-------------\n";
		// GrowingTreeSolver_t growingtree_solver(g, start, goal);
		// t.reset();
		// std::cout << "Start to compute time\n";
		// growingtree_solver.GrowingTreeSearch();
		// t1 = t.elapsed();
		// std::cout << "GrowTree time: " << t1 << " seconds\n\n";
		// //growingtree_solver.write_solution(file_dir4, t1);
	}

	return 0;
}