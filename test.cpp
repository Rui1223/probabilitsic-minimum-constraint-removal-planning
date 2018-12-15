/*test whether the fixedlabel solver works*/
#include "LabeledGraph.hpp"
#include "FixedLabelSolver.hpp"

#include <cstdio>
#include <iostream>
#include <queue>



int main()
{
	LabeledGraph_t g(2,6);
	g.cal_labelMap();
	g.print_labelMap();
	int start = 0;
	int goal = 5;
	FixedLabelSolver_t fixedlabel_solver(g, start, goal);
	fixedlabel_solver.fixedLabel_search();

	return 0;

}