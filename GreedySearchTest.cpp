/*This test file runs greedy search on a toy problem*/

#include "LabeledGraph.hpp"
#include "PmcrGreedySolver.hpp"

#include <cstdio>
#include <iostream>
#include <queue>



int main()
{
	// Problem input
	//LabeledGraph_t g(3, 3);
	LabeledGraph_t g(5, 5, 3);
	int start = 0;
	int goal = 24;
	// Call the search algorithm
	PmcrGreedySolver_t pmcr_solver(g, start, goal);
	pmcr_solver.greedy_search();
	return 0;

}

