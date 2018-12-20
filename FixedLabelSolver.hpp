/*This hpp files declare a solver used to solve weighted MCR problem under the assumption
  that each obstacle has fixed number of potential pose (equivalently, the number of labels
  is fixed in a MinLP problem)*/

#include "LabeledGraph.hpp"


class FixedLabelSolver_t
{
	LabeledGraph_t m_lgraph;
	int m_start;
	int m_goal;

	std::vector<int> m_path;
	std::vector<int> m_currentLabels;
	double m_currentWeight;

public:
	FixedLabelSolver_t() {}
	FixedLabelSolver_t(LabeledGraph_t &g, int start, int goal);
	~FixedLabelSolver_t() {}

	// The function to search for a path in a fixed label scenario
	void fixedLabel_search();

	// The function to check whether a input set of labels is a subset of the m_currentLabel
	bool check_subset(std::vector<int> labels);

	// The function to search on a subgraph with a certain set of labels being looped on
	bool BFSearch();

	// The function to back track the path
	void backtrackPath(std::vector<int> &parents);

	// The function to print the path
	void print_path();

};

std::ostream& operator<<(std::ostream &out, const std::vector<int> &v);
