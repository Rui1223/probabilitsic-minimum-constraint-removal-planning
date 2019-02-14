/*This hpp files declare a solver used to solve weighted MCR problem under the assumption
  that each obstacle has fixed number of potential pose (equivalently, the number of labels
  is fixed in a MinLP problem)*/

#ifndef FIXEDLABELSOLVER_H
#define FIXEDLABELSOLVER_H

#include <cstring>

#include "HeuristicSearchSolver.hpp"
#include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
#include "ConnectedNonOverlapGraph.hpp"

// Declaring the type of Predicate that accept two pairs and return a bool
typedef std::function<bool(std::pair<std::vector<int>, double>, 
	std::pair<std::vector<int>, double>)> Comparator1;

class FixedLabelSolver_t
{
	ConnectedGraph_t m_lgraph;
	int m_start;
	int m_goal;

	std::vector<int> m_currentLabels;
	double m_currentWeight;
	std::vector<int> m_path;

	HeuristicSearchSolver_t m_heuristic_search_solver;

public:
	FixedLabelSolver_t() {}
	FixedLabelSolver_t(ConnectedGraph_t &g, int start, int goal);

	~FixedLabelSolver_t() {}

	std::vector<int> cal_sgLabel(int start, int goal);
	std::set<std::pair<std::vector<int>, double>, Comparator1> cal_subLabelMap(std::vector<int>);

	// The function to search for a path in a fixed label scenario
	void fixedLabel_search();

	// The function to perform heuristic search on a subgraph 
	// with a certain set of labels being looped on
	bool HeuristicSearch();

	// The function to write in the solution
	void write_solution(std::string file_dir, double t);

	// getters
	std::vector<int> getCurrentLabels() { return m_currentLabels; }
	double getCurrentWeight() { return m_currentWeight; }

};

std::vector<int> label_intersection(std::vector<int>, std::vector<int>);
std::vector<int> label_union(std::vector<int>, std::vector<int>);
bool check_subset(std::vector<int> set, std::vector<int> subset);

#endif
