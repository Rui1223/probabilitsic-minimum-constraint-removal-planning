/*This cpp files contains main to test greedy search and fixedLabel Search together*/

#include "LabeledGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "FixedLabelSolver.hpp"
#include "Timer.hpp"

#include <iostream>
#include <cstdlib> // for std::srand()
#include <ctime>

int main()
{
	std::srand(static_cast<unsigned int>(std::time(nullptr)));

	LabeledGraph_t g(20, 20, 5);
	int start = 0;
	int goal = 399;
	g.write_graph();

	Timer t;
	std::cout << "----------start the fixedLabel search-------------\n";
	FixedLabelSolver_t fixedlabel_solver(g, start, goal);
	fixedlabel_solver.fixedLabel_search();
	std::cout << "\nTimer elapsed: " << t.elapsed() << " seconds\n";
	fixedlabel_solver.write_solution();

	t.reset();
	std::cout << "----------start the greedy search-------------\n";
	PmcrGreedySolver_t pmcr_solver(g, start, goal);
	pmcr_solver.greedy_search();
	std::cout << "\nTimer elapsed: " << t.elapsed() << " seconds\n";
	pmcr_solver.write_solution();

	return 0;

}