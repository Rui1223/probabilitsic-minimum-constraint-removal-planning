/*This hpp files declare a solver used to solve weighted MCR problem under the assumption
  that each obstacle has fixed number of potential pose (equivalently, the number of labels
  is fixed in a MinLP problem)*/

#ifndef FIXEDLABELSOLVER_H
#define FIXEDLABELSOLVER_H

#include "LabeledGraph.hpp"


class FixedLabelSolver_t
{
	LabeledGraph_t m_lgraph;
	int m_start;
	int m_goal;

	std::vector<int> m_currentLabels;
	double m_currentWeight;

public:
	FixedLabelSolver_t() {}
	FixedLabelSolver_t(LabeledGraph_t &g, int start, int goal);
	~FixedLabelSolver_t() {}

	// The function to search for a path in a fixed label scenario
	void fixedLabel_search(int n_time);

	// The function to perform heuristic search on a subgraph 
	// with a certain set of labels being looped on
	bool HeuristicSearch(int n_time);

};

#endif
