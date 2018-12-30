/*The cpp file do several experiements on greedy ahnd fixedLabel search*/
/*save graphs and several corresponding solutions, respectively*/

#include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "FixedLabelSolver.hpp"
#include "Timer.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <string> // std::string, std::to_string

// int main()
// {
// 	Timer t;
// 	std::srand(std::time(0));
// 	int nExperiments = 5;
// 	for (int ii = 0; ii < nExperiments; ii++)
// 	{
// 		// generate a grpah
// 		LabeledGraph_t g(15,15,5, 0.4);
// 		int start = random_generate_integer(0, 15*15-1);
// 		int goal = random_generate_integer(0, 15*15-1);
// 		while (start == goal)
// 		{
// 			int start = random_generate_integer(0, 15*15-1);
// 			int goal = random_generate_integer(0, 15*15-1);
// 		}
// 		g.write_graph(ii);

// 		// work out on solutions
// 		t.reset();
// 		std::cout << "----------start the fixedLabel search-------------\n";
// 		FixedLabelSolver_t fixedlabel_solver(g, start, goal);
// 		fixedlabel_solver.fixedLabel_search();
// 		std::cout << "\nTimer elapsed: " << t.elapsed() << " seconds\n";
// 		fixedlabel_solver.write_solution(ii);

// 		t.reset();
// 		std::cout << "----------start the greedy search-------------\n";
// 		PmcrGreedySolver_t pmcr_solver(g, start, goal);
// 		pmcr_solver.greedy_search();
// 		std::cout << "\nTimer elapsed: " << t.elapsed() << " seconds\n";
// 		pmcr_solver.write_solution(ii);
// 	}

// 	return 0;
// }

/*
int main()
{
	std::srand(static_cast<unsigned int>(std::time(nullptr)));
	Timer t;

	LabeledGraph_t g(25, 25, 5, 0.5);
	int start = random_generate_integer(0, 15*15-1);
	int goal = random_generate_integer(0, 15*15-1);
	while (start == goal)
	{
		int start = random_generate_integer(0, 15*15-1);
		int goal = random_generate_integer(0, 15*15-1);
	}
	g.write_graph(0);

	// std::cout << "----------start the fixedLabel search-------------\n";
	// FixedLabelSolver_t fixedlabel_solver(g, start, goal);
	// t.reset();
	// fixedlabel_solver.fixedLabel_search(0);
	// std::cout << "\nTimer elapsed: " << t.elapsed() << " seconds\n";

	// std::cout << "----------start the greedy search-------------\n";
	// PmcrGreedySolver_t pmcr_solver(g, start, goal);
	// t.reset();
	// pmcr_solver.greedy_search();
	// std::cout << "\nTimer elapsed: " << t.elapsed() << " seconds\n";
	// pmcr_solver.write_solution(0);

	return 0;

}
*/

int main()
{
	std::srand(static_cast<unsigned int>(std::time(nullptr)));
	Timer t;

	int gridSize = 10;
	int nLabels = 5;
	double labelCoverage = 0.5;
	ConnectedGraph_t g(gridSize, gridSize, nLabels, labelCoverage);
	int start = random_generate_integer(0, gridSize*gridSize-1);
	int goal = random_generate_integer(0, gridSize*gridSize-1);
	while (start == goal)
	{
		int start = random_generate_integer(0, gridSize*gridSize-1);
		int goal = random_generate_integer(0, gridSize*gridSize-1);
	}
	g.write_graph(0);
}
