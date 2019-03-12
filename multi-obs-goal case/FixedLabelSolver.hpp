/*This hpp files declare a solver used to solve weighted MCR problem under the assumption
  that each obstacle has fixed number of potential pose (equivalently, the number of labels
  is fixed in a MinLP problem)*/

#ifndef FIXEDLABELSOLVER_H
#define FIXEDLABELSOLVER_H

#include <cassert>
#include <cmath>
#include <queue>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cstring> // std::string, std::to_string
#include <map>
#include <cassert>
#include <functional>
#include <set>
// #include "HeuristicSearchSolver.hpp"
// #include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
// #include "ConnectedNonOverlapGraph.hpp"
// #include "ToyGraph.hpp"

// Declaring the type of Predicate that accept two pairs and return a bool
// typedef std::function<bool(std::pair<std::vector<int>, double>, 
// 	std::pair<std::vector<int>, double>)> Comparator1;

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

class FixedLabelSolver_t
{
	// Input that a fixedLabel solver needs
	ConnectedGraph_t m_lgraph;
	int m_start;
	std::vector<int> m_goalSet; // the ids of all goal nodes
	int m_targetObs; //obs idx for target
	std::vector<int> m_targetPoses;
	int m_optimalGoal;
	int m_optimalPose;
	double m_optimalSurvival;
	std::vector<int> m_optimalLabels;

	std::vector<int> m_currentLabels;
	double m_currentSurvival;
	std::vector<std::vector<int>> m_paths;

	//HeuristicSearchSolver_t m_heuristic_search_solver;
	std::priority_queue<HeuristicNode_t*, std::vector<HeuristicNode_t*>, 
					HeuristicNode_comparison> m_open;
	std::vector<HeuristicNode_t*> m_closed;
	std::vector<bool> m_expanded;

	// 3 key variables
	double m_MaxSurvival;
	double m_highestSuccess;
	double m_lowestReachability;
	std::vector<double> m_pathSuccess;

	// some unique features for the fixedLabel search
	// all label combinations
	int m_nTotallabels;
	// for each label idx, we store the obs idx it belongs to and its corresponding weight
	std::map<int, std::pair<int, double>> m_labelWeights;
	std::vector<std::vector<int>> m_labelCombinations;
	std::vector<std::pair<std::vector<int>, double>> m_labelMap;

	int m_k;

public:
	FixedLabelSolver_t() {}
	FixedLabelSolver_t(ConnectedGraph_t &g);

	// destructor used to free space before exit
	~FixedLabelSolver_t();

	// std::vector<int> cal_sgLabel(int start, int goal);
	// std::vector<std::pair<std::vector<int>, double>> cal_subLabelMap(std::vector<int>);

	// The function to search for a path in a fixed label scenario
	void fixedLabel_search();

	// The function to perform heuristic search on a subgraph 
	// with a certain set of labels being looped on
	void HeuristicSearch();

	// The functopm to compute the f,g,h value of a node(indx) for a given start & goal
	std::vector<int> computeFGH(int indx);

	// The function to prune unqualified goals in goalSet
	void prune_goalSet();

	// The function to back track the path
	void back_track_path();

	// The function to print current goalSet
	void print_goalSet();

	// The function to check whether a input set of labels is a subset of the m_currentLabels
	bool check_subset(std::vector<int> labels);

	// The function to write in the solution
	void write_solution(std::string file_dir, double t);

	// function to compute the survivability for a single set of labels
	//double compute_survival_currentLabels(std::vector<int> labels);

	// function to compute survivability for a powerset
	std::vector<double> compute_survival();

	// function to compute a powerset given a set of labels
	void cal_powerSet();

	// function to calculate labelMap
	void cal_labelMap();

	// function to print labelMap for test purpose so far
	void print_labelMap();	

	// getters
	std::vector<int> getCurrentLabels() { return m_currentLabels; }
	double getCurrentSurvival() { return m_currentSurvival; }
	double getHighestSuccess() { return m_highestSuccess; }

};

std::vector<int> label_intersection(std::vector<int>, std::vector<int>);
std::vector<int> label_union(std::vector<int>, std::vector<int>);
bool check_subset(std::vector<int> set, std::vector<int> subset);

#endif
