/*This test file is for test purpose*/

#include "LabeledGraph.hpp"
#include "PmcrGreedySolver.hpp"

#include <cstdio>
#include <iostream>
#include <queue>



int main()
{
	// Problem input
	//LabeledGraph_t g(3, 3);
	LabeledGraph_t g(2, 6);
	int start = 0;
	int goal = 5;
	// Call the search algorithm
	PmcrGreedySolver_t pmcr_solver(g, start, goal);
	pmcr_solver.greedy_search();
	return 0;

}

