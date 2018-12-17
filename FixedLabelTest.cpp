/*test whether the fixedlabel solver works*/
#include "LabeledGraph.hpp"
#include "FixedLabelSolver.hpp"

#include <cstdio>
#include <iostream>
#include <queue>
#include <cstdlib>
#include <ctime>



int main()
{
	std::srand(static_cast<unsigned int>(std::time(nullptr)));

	LabeledGraph_t g(5, 5, 3);
	int start = 0;
	int goal = 24;
	FixedLabelSolver_t fixedlabel_solver(g, start, goal);
	fixedlabel_solver.fixedLabel_search();

	return 0;

}