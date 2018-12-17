/*This cpp files contains main to test greedy search and fixedLabel Search together*/

#include "LabeledGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "FixedLabelSolver.hpp"

#include <cstdio>
#include <iostream>
#include <queue>
#include <cstdlib>
#include <ctime>

int main()
{
	std::srand(static_cast<unsigned int>(std::time(nullptr)));

	LabeledGraph_t g(50, 50, 3);
	int start = 0;
	int goal = 2499;
	FixedLabelSolver_t fixedlabel_solver(g, start, goal);
	fixedlabel_solver.fixedLabel_search();
	std::cout << "Now start the greedy search\n";
	PmcrGreedySolver_t pmcr_solver(g, start, goal);
	pmcr_solver.greedy_search();

	return 0;

}