/* This hpp file define the MCR problem and the greedy search algorithm applied to this problem */

#ifndef PMCRGREEDYSOLVER_H
#define PMCRGREEDYSOLVER_H

#include <queue>
#include <cstring>

// #include "LabeledGraph.hpp"
// #include "ConnectedGraph.hpp"
// #include "ConnectedNonOverlapGraph.hpp"
#include "ToyGraph.hpp"
#include "PmcrNode.hpp"

struct PmcrNode_comparison
{
	bool operator()(const PmcrNode_t* a, const PmcrNode_t* b) 
	{
		if (a->getSurvival() == b->getSurvival())
		{
			return (a->getH()) > (b->getH());
		}
		else
			return (a->getSurvival()) < (b->getSurvival());
	}
};


class PmcrGreedySolver_t
{
	// Input that a greedy solver needs
	ToyGraph_t m_lgraph; 
	int m_start; // the id of the start node
	int m_goal; // the id of the goal node

	std::vector<int> m_currentLabels;
	double m_currentSurvival;
	std::vector<double> m_highestSurvival;
	std::vector<int> m_path;

	std::priority_queue<PmcrNode_t*, std::vector<PmcrNode_t*>, PmcrNode_comparison> m_open;
	std::vector<PmcrNode_t*> m_closed;
	std::vector<bool> m_expanded;

public:
	PmcrGreedySolver_t(ToyGraph_t &g, int start, int goal);
	~PmcrGreedySolver_t();
	void greedy_search();
	void back_track_path();
	std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2);
	// The functopm tp compute the heuristic value of a node of a given index
	int computeH(int indx);
	//bool check_prune(int neighborID, double weights);
	//bool search_closedList(int neighborID, double weights, bool isPrune);
	//bool search_openList(int neighborID, double weights, bool isPrune);
	//void push_virtualOpen();
	void print_path();
	void write_solution(std::string file_dir, double t);
	void print_closedList();

	//getters
	double getCurrentSurvival() { return m_currentSurvival; }
};

#endif