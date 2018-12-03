/*This test file is for test purpose*/

#include "LabeledGraph.hpp"
#include "PmcrGreedySolver.hpp"

#include <cstdio>
#include <iostream>
#include <queue>

int main()
{
	// Problem input
	LabeledGraph_t g(3);
	int start = 0;
	int goal = 8;
	// Call the search algorithm
	PmcrGreedySolver_t pmcr_solver(g, start, goal);
	pmcr_solver.greedy_search();

}

/*for testing*/
// int main()
// {
// 	std::vector<double> labelweights{0.0, 0.1, 0.6, 0.3}; 

// 	std::priority_queue<PmcrNode_t> open;
	
// 	// Create 5 nodes to test whether priority queue works or not
// 	PmcrNode_t n0 = PmcrNode_t(0, {0}, nullptr, labelweights);
// 	PmcrNode_t n1 = PmcrNode_t(1, {1}, &n0, labelweights);
// 	PmcrNode_t n2 = PmcrNode_t(2, {1,3}, &n1, labelweights);
// 	PmcrNode_t n3 = PmcrNode_t(3, {2}, &n2, labelweights);
// 	PmcrNode_t n4 = PmcrNode_t(4, {1,2,3}, &n3, labelweights);

// 	open.push(n0);
// 	open.push(n1);
// 	open.push(n2);
// 	open.push(n3);
// 	open.push(n4);

// 	while (!open.empty())
// 	{
// 		PmcrNode_t p = open.top();
// 		open.pop();
// 		p.print();
// 	}

// 	return 0;
// }
