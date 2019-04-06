#ifndef MAXLIKELIHOODSOLVER_H
#define MAXLIKELIHOODSOLVER_H

#include <queue>
#include <cstring>

#include "ConnectedGraph.hpp"
#include "PmcrNode.hpp"

struct PmcrNode_comparison_maxLikelihood
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

class MaxLikelihoodSolver_t
{
	// Input that a greedy solver needs
	// ConnectedGraph_t m_lgraph; 
	int m_col;
	int m_targetObs; //obs idx for target
	int m_nlabelsPerObs;
	int m_nObstacles;

	std::vector<int> m_currentLabels;
	double m_currentSurvival;
	std::vector<double> m_highestSurvival; // keep record of highest survivability for all nodes

	std::vector<int> m_path;
	std::priority_queue<PmcrNode_t*, std::vector<PmcrNode_t*>, PmcrNode_comparison_maxLikelihood> m_open;
	std::vector<PmcrNode_t*> m_closed;
	std::vector<bool> m_expanded;
	std::vector<int> m_G;

	// replanning things
	int m_start; // the id of the start node
	std::map<int, std::pair<int, double>> m_labelWeights;
	std::vector<int> m_goalSet; // the ids of all goal nodes
	std::vector<int> m_targetPoses;

	bool m_solvable;

public:
	MaxLikelihoodSolver_t(ConnectedGraph_t &g, int start, std::vector<int> goalSet, 
			std::vector<int> targetPoses, std::map<int, std::pair<int, double>> labelWeights);
	~MaxLikelihoodSolver_t();
	void MaxLikelihood_search(ConnectedGraph_t &g);

	void back_track_path();
	std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2);	
	// The function to compute the h value of a node(indx) for a given goal
	int computeH(int indx);
	void write_solution(std::string file_dir, double t);
	void print_closedList();

	double getCurrentSurvival() { return m_currentSurvival; }
	std::vector<int> getmPath() { return m_path; }

	int getmStart() { return m_start; }
	std::map<int, std::pair<int, double>> getLabelWeights() { return m_labelWeights; }
	bool getIsSolvable() { return m_solvable; }

	// function to compute the survivability for a single set of labels
	double compute_survival_currentLabels(std::vector<int> labels);

	void MaximumLikelihood_labelWeights();
	// void pick_goal();

};

#endif