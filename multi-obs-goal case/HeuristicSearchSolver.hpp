/*The hpp file declares a heuristic search solver used in fixed label search algorithm*/
#ifndef HEURISTICSEARCHSOLVER_H
#define HEURISTICSEARCHSOLVER_H

// #include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
// #include "ConnectedNonOverlapGraph.hpp"
// #include "ToyGraph.hpp"

struct HeuristicNode_t
{
	int m_id;
	std::vector<int> m_FGH;
	HeuristicNode_t *m_parent;
	HeuristicNode_t(int id, std::vector<int> FGH, HeuristicNode_t *parent)
		: m_id(id), m_FGH(FGH), m_parent(parent) {}
};

struct HeuristicNode_comparison
{
	bool operator()(const HeuristicNode_t* a, const HeuristicNode_t* b) 
	{
		if (a->m_FGH[0] == b->m_FGH[0])
		{
			return (a->m_FGH[2]) > (b->m_FGH[2]);
		}
		return (a->m_FGH[0]) > (b->m_FGH[0]);
	}
};

class HeuristicSearchSolver_t
{
	ConnectedGraph_t m_lgraph;
	int m_start;
	int m_goal;
	std::vector<int> m_currentLabels;
	double m_currentSurvival;

	std::priority_queue<HeuristicNode_t*, std::vector<HeuristicNode_t*>, 
					HeuristicNode_comparison> m_open;
	std::vector<HeuristicNode_t*> m_closed;
	std::vector<int> m_path;

public:
	// constructor
	HeuristicSearchSolver_t() {}
	HeuristicSearchSolver_t(ConnectedGraph_t &g, int start, int goal);

	bool Heuristic_search();

	// The function to compute the f,g,h value of a node of a given index based on start & goal
	std::vector<int> computeFGH(int indx);

	// The function to check whether a input set of labels is a subset of the m_currentLabels
	bool check_subset(std::vector<int> labels);

	// The function to back track the path
	void back_track_path();

	void print_path();

	// The function to print the closedList for testing purposes
	void print_closedList();

	// getters
	double getCurrentSurvival() { return m_currentSurvival; }
	std::vector<int> getCurrentLabels() { return m_currentLabels; }
	std::vector<int> getPath() { return m_path; }

	// setters
	void setCurrentLabels(std::vector<int> labels) { m_currentLabels = labels; }
	void setCurrentSurvival(double currentSurvival) { m_currentSurvival = currentSurvival; }

	// destructor used to free space before exit
	~HeuristicSearchSolver_t();	
};

std::ostream& operator<<(std::ostream &out, const std::vector<int> &v);

#endif