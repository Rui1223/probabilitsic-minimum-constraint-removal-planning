/*The hpp file declares a heuristic search solver used in fixed label search algorithm*/
#ifndef HEURISTICSEARCHSOLVER_H
#define HEURISTICSEARCHSOLVER_H

#include "ConnectedGraph.hpp"
#include "LabeledGraph.hpp"

struct HeuristicNode_t
{
	int m_id;
	int m_H;
	HeuristicNode_t *m_parent;
	HeuristicNode_t(int id, int H, HeuristicNode_t *parent)
		: m_id(id), m_H(H), m_parent(parent) {}
};

struct HeuristicNode_comparison
{
	bool operator()(const HeuristicNode_t* a, const HeuristicNode_t* b) 
	{
		return (a->m_H) > (b->m_H);
	}
};

class HeuristicSearchSolver_t
{
	LabeledGraph_t m_lgraph;
	int m_start;
	int m_goal;
	std::vector<int> m_currentLabels;
	double m_currentWeight;

	std::priority_queue<HeuristicNode_t*, std::vector<HeuristicNode_t*>, 
					HeuristicNode_comparison> m_open;
	std::vector<HeuristicNode_t*> m_closed;
	std::vector<int> m_path;

public:
	// constructor
	HeuristicSearchSolver_t(LabeledGraph_t &g, int start, int goal, std::vector<int> &l, double w);

	bool Heuristic_search();

	// The functopm tp compute the heuristic value of a node of a given index
	int computeH(int indx);

	// The function to check whether a input set of labels is a subset of the m_currentLabel
	bool check_subset(std::vector<int> labels);

	// The function to back track the path
	void back_track_path();

	void print_path();

	// The function to print the closedList for testing purposes
	void print_closedList();

	// getters
	double getCurrentWeight() { return m_currentWeight; }
	std::vector<int> getCurrentLabels() { return m_currentLabels; }
	std::vector<int> getPath() { return m_path; }

	// setters
	// void setCurrentLabels(std::vector<int> labels) { m_currentLabels = labels; }
	// void setCurrentWeight(double currentWeight) { m_currentWeight = currentWeight; }

	// destructor used to free space before exit
	~HeuristicSearchSolver_t();	
};

std::ostream& operator<<(std::ostream &out, const std::vector<int> &v);

#endif