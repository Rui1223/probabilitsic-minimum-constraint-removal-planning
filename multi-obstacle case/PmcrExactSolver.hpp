/* This hpp file define the probabilistic MCR problem and 
the exact search algorithm applied to this problem */

#ifndef PMCREXACTSOLVER_H
#define PMCREXACTSOLVER_H

#include <queue>
#include <cstring>

#include "ConnectedGraph.hpp"
// #include "ToyGraph.hpp"
#include "PmcrNode.hpp"

struct PmcrNode_comparison1
{
	bool operator()(const PmcrNode_t* a, const PmcrNode_t* b) 
	{
		if (a->getSurvival() == b->getSurvival())
		{
			if (a->getF() == b->getF())
			{
				return (a->getH()) > (b->getH());
			}
			else
				return (a->getF()) > (b->getF());
		}
		else
			return (a->getSurvival()) < (b->getSurvival());
	}
};


class PmcrExactSolver_t
{
	// Input that a exact solver needs
	// ConnectedGraph_t m_lgraph; 
	int m_start; // the id of the start node
	int m_goal; // the id of the goal node
	int m_col;

	std::vector<int> m_currentLabels;
	double m_currentSurvival;
	//std::vector<double> m_highestSurvival;
	std::vector<int> m_path;

	std::priority_queue<PmcrNode_t*, std::vector<PmcrNode_t*>, PmcrNode_comparison1> m_open;
	std::vector<PmcrNode_t*> m_closed;
	//std::vector<bool> m_expanded;
	std::vector<int> m_G;
	std::vector<std::vector<std::vector<int>>> m_recordSet;
	std::vector<bool> m_visited;

public:
	PmcrExactSolver_t(ConnectedGraph_t &g);
	~PmcrExactSolver_t();
	void exact_search(ConnectedGraph_t &g);
	void back_track_path();
	void print_path();
	std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2);
	// The functopm to compute the h value of a node(indx) for a given goal
	int computeH(int indx);
	//bool check_prune(int neighborID, double weights);
	void write_solution(std::string file_dir, double t);
	void print_closedList();

	bool check_superset(int, std::vector<int>);

	//getters
	double getCurrentSurvival() { return m_currentSurvival; }
};

bool check_subset(std::vector<int>, std::vector<int>);

#endif