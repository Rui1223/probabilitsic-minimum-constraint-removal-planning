/* This hpp file define the multi-goal probabilistic MCR problem and
the greedy search algorithm applied to this problem */

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
	// ConnectedGraph_t m_lgraph; 
	int m_start; // the id of the start node
	std::vector<int> m_goalSet; // the ids of all goal nodes
	std::vector<int> m_goalSetD;
	int m_col;
	std::map<int, std::pair<int, double>> m_labelWeights;


	int m_targetObs; //obs idx for target
	std::vector<int> m_targetPoses;
	std::vector<int> m_targetPosesD;

	int m_optimalGoal;
	int m_optimalPose;
	double m_optimalSurvival;
	std::vector<int> m_optimalLabels;
	std::vector<int> m_optimalPath;

	std::vector<int> m_currentLabels;
	double m_currentSurvival;
	std::vector<double> m_highestSurvival; // keep record of highest survivability for all nodes
	std::vector<std::vector<int>> m_paths;

	std::priority_queue<PmcrNode_t*, std::vector<PmcrNode_t*>, PmcrNode_comparison> m_open;
	std::vector<PmcrNode_t*> m_closed;
	std::vector<bool> m_expanded;
	std::vector<int> m_G;

	// 3 key variables
	double m_MaxSurvival;
	double m_highestSuccess;
	double m_lowestReachability;
	std::vector<double> m_pathSuccess;

public:
	PmcrGreedySolver_t(ConnectedGraph_t &g);
	~PmcrGreedySolver_t();
	void greedy_search(ConnectedGraph_t &g);

	void prune_goalSet(ConnectedGraph_t &g);
	void back_track_path();
	std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2);
	// The function to compute the h value of a node(indx) for a given goal
	int computeH(int indx);
	// bool check_prune(int neighborID, double weights);
	// void print_currentGoalSet();
	void write_solution(std::string file_dir, double t);
	void print_closedList();

	//getters
	double getCurrentSurvival() { return m_currentSurvival; }
	double getHighestSuccess() { return m_highestSuccess; }
	double getOptimalSurvival() { return m_optimalSurvival; }
	std::vector<int> getOptimalPath() { return m_optimalPath; }

	// update the labelWeights after execute the path
	void updateLabelWeights(int label, bool mode);
	// update start in order to replan
	void updateStart(int start) { m_start = start; }
};

#endif