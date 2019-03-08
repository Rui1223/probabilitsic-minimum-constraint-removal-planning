/* This hpp file define the MCR problem and the greedy search algorithm applied to this problem */

#ifndef PMCRGREEDYSOLVER_H
#define PMCRGREEDYSOLVER_H

#include <queue>
#include <cstring>

#include "ConnectedGraph.hpp"
#include "PmcrNode.hpp"

struct PmcrNode_comparison
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


class PmcrGreedySolver_t
{
	// Input that a greedy solver needs
	ConnectedGraph_t m_lgraph; 
	int m_start; // the id of the start node
	std::vector<int> m_goalSet; // the ids of all goal nodes
	int m_targetObs; //obs idx for target
	std::vector<int> m_targetPoses;
	int m_optimalGoal;
	int m_optimalPose;

	std::vector<int> m_currentLabels;
	double m_currentSurvival;
	std::vector<double> m_highestSurvival;
	std::vector<std::vector<int>> m_paths;

	std::priority_queue<PmcrNode_t*, std::vector<PmcrNode_t*>, PmcrNode_comparison> m_open;
	std::vector<PmcrNode_t*> m_closed;
	std::vector<bool> m_expanded;

	// 3 key variables
	double m_MaxSurvival;
	double m_highestSuccess;
	double m_lowestReachability;
	std::vector<double> m_pathSuccess;

public:
	PmcrGreedySolver_t(ConnectedGraph_t &g);
	~PmcrGreedySolver_t();
	void greedy_search();
	void prune_goalSet();
	void back_track_path();
	std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2);
	// The functopm to compute the f,g,h value of a node(indx) for a given start & goal
	std::vector<int> computeFGH(int indx);
	//bool check_prune(int neighborID, double weights);
	//void print_path();
	void write_solution(std::string file_dir, double t);
	void print_closedList();

	//getters
	double getCurrentSurvival() { return m_currentSurvival; }
};

#endif