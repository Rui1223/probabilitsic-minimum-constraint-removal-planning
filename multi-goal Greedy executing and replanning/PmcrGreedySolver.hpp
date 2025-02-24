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
	int m_col;
	int m_targetObs; //obs idx for target
	int m_nlabelsPerObs;
	int m_nObstacles;


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

	// replanning things
	int m_start; // the id of the start node
	std::map<int, std::pair<int, double>> m_labelWeights;
	std::vector<int> m_goalSet; // the ids of all goal nodes
	std::vector<int> m_targetPoses;

	bool m_solvable;

public:
	PmcrGreedySolver_t(ConnectedGraph_t &g, int start, std::vector<int> goalSet, 
			std::vector<int> targetPoses, std::map<int, std::pair<int, double>> labelWeights);
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
	double getLabelWeight(int indx) { return m_labelWeights[indx].second; }
	int getmStart() { return m_start; }
	std::vector<int> getGoalSet() { return m_goalSet; }
	std::vector<int> getTargetPose() { return m_targetPoses; }
	std::map<int, std::pair<int, double>> getLabelWeights() { return m_labelWeights; }
	bool getIsSolvable() { return m_solvable; }

	// function to compute the survivability for a single set of labels
	double compute_survival_currentLabels(std::vector<int> labels);
	
};

#endif