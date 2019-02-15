/*This cpp file test the effectiveness of the GrowingTree algorithm on a toy problem*/

// #include "LabeledGraph.hpp"
// #include "ConnectedGraph.hpp"
// #include "ConnectedNonOverlapGraph.hpp"
#include "ToyGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "FixedLabelSolver.hpp"
// #include "GrowingTreeSolver.hpp"
#include "Timer.hpp"

#include <cstdio>
#include <cassert>
#include <cmath>  // pow(), sqrt()
#include <iostream> 
#include <fstream> // stream class to write on files
#include <cstdio> // printf() 
#include <vector> // std::vector
#include <algorithm> // std::set_union, std::sort
#include <bitset> // for bitwise operation
#include <map> // std::map
#include <functional>
#include <set>
#include <cstdlib> // for std::srand()
#include <string> // std::string, std::to_string
#include <queue>
#include <random>

int main(int argc, char** argv)
{
	// Please specify #labels per label
	std::vector<int> nlabelsPerObs;
	nlabelsPerObs.push_back(2);
	nlabelsPerObs.push_back(2);
	ToyGraph_t toy_graph(2, 6, nlabelsPerObs);

	int start = 0;
	int goal = 5;
	// try greedy search approach
	std::cout << "**************Greedy Search*************";
	PmcrGreedySolver_t pmcr_greedy_solver(toy_graph, start, goal);
	Timer t;
	pmcr_greedy_solver.greedy_search();
	std::cout << "Time for greedy search on this easy toy problem: " << t.elapsed() << " seconds\n";


	std::cout << "************FixedLabel Search*************";
	FixedLabelSolver_t fixedlabel_solver(toy_graph, start, goal);
	t.reset();
	fixedlabel_solver.fixedLabel_search();
	std::cout << "Time for FixedLabel search on this easy toy problem: " << t.elapsed() << " seconds\n";

	return 0;
}
