/*This cpp file test the effectiveness of the GrowingTree algorithm on a toy problem*/

#include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
#include "ConnectedNonOverlapGraph.hpp"
#include "ToyGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "FixedLabelSolver.hpp"
#include "GrowingTreeSolver.hpp"
#include "Timer.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <string> // std::string, std::to_string

int main(int argc, char** argv)
{
	int nExperiments = 1;
	Timer t;
	double t1;

	// generate the toy problem
	ToyGraph_t g(2, 6, 3);
	int start = 0;
	int goal = 5;
	std::cout << "-------------start the GrowingTree search-------------\n";
	GrowingTreeSolver_t growingtree_solver(g, start, goal);
	t.reset();
	growingtree_solver.GrowingTreeSearch();
	t1 = t.elapsed();
	std::cout << "\nGrowingTree time: " << t1 << " seconds\n";

	return 0;

}
