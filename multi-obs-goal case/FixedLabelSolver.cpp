/*This cpp files defines a solver used to solve weighted MCR problem under the assumption
  that each obstacle has fixed number of potential pose (equivalently, the number of labels
  is fixed in a MinLP problem)*/
/*full implementation*/

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

#include "FixedLabelSolver.hpp"
#include "HeuristicSearchSolver.hpp"
// #include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
// #include "ConnectedNonOverlapGraph.hpp"
//#include "ToyGraph.hpp"
#include "Timer.hpp"

// Comparator compFunctor1 = 
// 		[](std::pair<std::vector<int>, double> elem1, std::pair<std::vector<int>, double> elem2)
// 		{
// 			return elem1.second <= elem2.second; // compare weights, prefer less weight  
// 		};
// Driver function to sort the vector elements 
// by second element of pairs 
// bool sortbysec1(const std::pair<std::vector<int>, double> &p1, 
// 				const std::pair<std::vector<int>, double> &p2)
// {
// 	return (p1.second >= p2.second);
// }

// Driver function to sort the vector elements 
// by second element of pairs 
bool sortbysec_fixedlabel(const std::pair<std::vector<int>, double> &p1, 
				const std::pair<std::vector<int>, double> &p2)
{
	return (p1.second < p2.second);
}

FixedLabelSolver_t::FixedLabelSolver_t(ConnectedGraph_t &g)
{
	Timer tt;
	tt.reset();
	m_lgraph = g;
	std::cout << "Time to load the graph for Fsolver: " << tt.elapsed() << " seconds\n\n";
	m_start = m_lgraph.getmStart();
	m_goalSet = m_lgraph.getmGoalSet();
	m_targetObs = m_lgraph.getmTargetObs();
	m_targetPoses = m_lgraph.getmTargetPoses();

	m_nTotallabels = m_lgraph.getnTotallabels();
	m_labelWeights = m_lgraph.getLabelWeights();

	// construct labelMap here.
	cal_labelMap();
	// print_labelMap();

	// Initialize 3 key variables we will be using
	m_highestSuccess = 0.0;
	m_lowestReachability = 0.0;
	// m_heuristic_search_solver = HeuristicSearchSolver_t(g, start, goal);
}

void FixedLabelSolver_t::fixedLabel_search()
{
	// Then for each set of labels and corresponding weight in the labelMap, construct a subgraph
	// which only contains edges which carry a subset of the current set of labels being looped on. 
	// An A* search is performed on the subgraph to see whether we can reach any one of the goal 
	// from the start. If it is, return the path to be the optimal so far. Otherwise
	// switch to next set of labels in the labelMap (with ascending order with respect to 
	// survivability).

	int k = 0;

	while (!m_goalSet.empty())
	{
		k++;

	}
}


void FixedLabelSolver_t::fixedLabel_search()
{
	// Then for each set of labels and corresponding weight in the labelMap, construct a subgraph
	// which only contains edges which carry a subset of the current set of labels being looped on. 
	// An A* search is performed on the subgraph to see whether we can reach any one of the goal 
	// from the start. If it is, return the path to be the optimal so far. Otherwise
	// switch to next set of labels in the labelMap (with ascending order with respect to survivability).
	int k = 1;
	for (int lc=m_labelMap.size()-1; lc>=0; lc--)
	{
		m_currentLabels = m_labelMap[lc].first;
		m_currentSurvival = m_labelMap[lc].second;
		//std::cout << "current set of labels: " << m_currentLabels << "\n";
		//std::cout << "currrent weight: " << m_currentWeight << "\n"; 
		std::cout << "start the " << k << "th search\n";

		bool goalFound = HeuristicSearch();
		//print_closedList();
		if (goalFound)
		{
			return;
		}
		k++;
		// otherwise switch to next set of labels in the labelMap (next pair)
	}

}

bool FixedLabelSolver_t::HeuristicSearch()
{
	// HeuristicSearch() performs a greedy Best-First search based on HeuristicSearchSolver 
	// with a label checking condition, which decides whether a newly propogated node 
	// should be added into the priority queue

	//HeuristicSearchSolver_t heuristic_search_solver(m_lgraph, m_start, m_goal,
													//m_currentLabels, m_currentWeight);
	m_heuristic_search_solver.setCurrentLabels(m_currentLabels);
	m_heuristic_search_solver.setCurrentSurvival(m_currentSurvival);
	bool goalFound = m_heuristic_search_solver.Heuristic_search();
	if (goalFound)
	{
		m_path = m_heuristic_search_solver.getPath();
	}
	return goalFound;
}



void FixedLabelSolver_t::write_solution(std::string file_dir, double t)
{
	std::ofstream file_(file_dir);
	if (file_.is_open())
	{
		file_ << m_start << " " << m_goal << " " << t << "\n";
		for (auto const &waypoint : m_path)
		{
			file_ << waypoint << " ";
		}
		file_ << "\n";
		// write the label and weight for the solution
		file_ << m_currentSurvival << "\n";
		
		if (!m_currentLabels.empty())
		{
			int pp = 0;
			while (pp < m_currentLabels.size()-1)
			{
				file_ << m_currentLabels[pp] << ",";
				pp++;
			}
			file_ << m_currentLabels[pp];
		}
		file_ << "\n";
		file_.close();
	}
}



void FixedLabelSolver_t::cal_labelMap()
{
	cal_powerSet();
	std::vector<double> survivalCombinations = compute_survival();

	for (int kkk=0; kkk < survivalCombinations.size(); kkk++)
	{
		m_labelMap.push_back(std::pair<std::vector<int>, double>(m_labelCombinations[kkk], 
																	survivalCombinations[kkk]));
	}

	sort(m_labelMap.begin(), m_labelMap.end(), sortbysec_fixedlabel);
}	

void FixedLabelSolver_t::cal_powerSet()
{
	// This function takes a set and then compute the powerset of the set
	// which is a vector of sets (std::vector<std::vector<int>>)

	// first determines the size of the powerset that you will have
	// for each one's derivation we will do bitwise operation
	int powerSet_size = pow(2, m_nTotallabels); // 2^n combinations
	for (int counter = 0; counter < powerSet_size; counter++)
	{
		std::vector<int> labels; // labels for a single combination
		for (int j=0; j < m_nTotallabels; j++)
		{
			if ( counter & (1<<j) )
			{
				labels.push_back(j);
			}
		}
		m_labelCombinations.push_back(labels);
	}
}

std::vector<double> FixedLabelSolver_t::compute_survival()
{
	std::vector<double> survivalCombinations;
	for (auto const &set : m_labelCombinations)
	{
		double survival = m_lgraph.compute_survival_currentLabels(set);
		survivalCombinations.push_back(survival);
	}

	return survivalCombinations;
}

void FixedLabelSolver_t::print_labelMap()
{
	printf("*********labelMap************\n");
	//Iterate over the set you just come up with
	for (auto const &e : m_labelMap)
	{
		std::cout << "<";
		for (auto const &l : e.first)
		{
			std::cout << l << ",";
		}
		std::cout << "> :\t\t";
		std::cout << e.second << std::endl;
	}
	printf("***************************\n");
}


// std::vector<int> FixedLabelSolver_t::cal_sgLabel(int start, int goal)
// {
// 	std::vector<int> start_neighbors = m_lgraph.getNodeNeighbors(start);
// 	std::vector<int> goal_neighbors = m_lgraph.getNodeNeighbors(goal);
// 	std::vector<std::vector<int>> start_setsLabels;
// 	std::vector<std::vector<int>> goal_setsLabels;
// 	for (auto const &neighbor : start_neighbors)
// 	{
// 		start_setsLabels.push_back(m_lgraph.getEdgeLabels(start, neighbor));
// 	}
// 	for (auto const &neighbor : goal_neighbors)
// 	{
// 		goal_setsLabels.push_back(m_lgraph.getEdgeLabels(goal, neighbor));
// 	}

// 	std::vector<int> start_labels = start_setsLabels[0];
// 	std::vector<int> goal_labels = goal_setsLabels[0];
// 	for (int kk=1; kk < start_setsLabels.size(); kk++)
// 	{
// 		start_labels = label_intersection(start_labels, start_setsLabels[kk]);
// 	}
// 	for (int kk=1; kk < goal_setsLabels.size(); kk++)
// 	{
// 		goal_labels = label_intersection(goal_labels, goal_setsLabels[kk]);
// 	}
// 	std::cout << "start_labels: " << start_labels << "\n";
// 	std::cout << "goal_labels: " << goal_labels << "\n";

// 	// So far we have compute start_labels and goal_labels
// 	// We need to compute the union of start_labels & goal_labels
// 	std::vector<int> sgLabels = label_union(start_labels, goal_labels);

// 	return sgLabels;
// }


// std::vector<std::pair<std::vector<int>, double>> FixedLabelSolver_t::cal_subLabelMap(
// 																		std::vector<int> sgLabels)
// {
// 	// std::map<std::vector<int>, double> sub_m;
// 	// for (auto const &e : m_lgraph.getLabelMap())
// 	// {
// 	// 	if (check_subset(e.first, sgLabels))
// 	// 	{
// 	// 		sub_m[e.first] = e.second;
// 	// 	}
// 	// }
// 	// std::set<std::pair<std::vector<int>, double>, Comparator> subLabelMap;
// 	// subLabelMap = std::set<std::pair<std::vector<int>, double>, Comparator>(
// 	// 	sub_m.begin(), sub_m.end(), compFunctor1);

// 	std::vector<std::pair<std::vector<int>, double>> subLabelMap;

// 	for (auto const &e: m_lgraph.getLabelMap())
// 	{
// 		if (check_subset(e.first, sgLabels))
// 		{
// 			subLabelMap.push_back(e);
// 		}
// 	}

// 	// sort (actually I don't need sort, since it has been sorted 
// 	// and by deletion the order is unchanged)
// 	//sort(m_labelMap.begin(), m_labelMap.end(), sortbysec1);

// 	return subLabelMap;
// }

std::vector<int> label_intersection(std::vector<int> v1, std::vector<int> v2)
{
	std::vector<int> s;
	std::sort(v1.begin(), v1.end());
	std::sort(v2.begin(), v2.end());
	std::set_intersection(v1.begin(),v1.end(),v2.begin(),v2.end(), back_inserter(s));
	return s;	
}

std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2)
{
	// sort the sets first before applying union operation
	std::sort(s1.begin(), s1.end());
	std::sort(s2.begin(), s2.end());

	// Declaring resultant vector for union
	std::vector<int> v(s1.size()+s2.size());
	// using function set_union() to compute union of 2
	// containers v1 and v2 and store result in v
	auto it = std::set_union(s1.begin(), s1.end(), s2.begin(), s2.end(), v.begin());

	// resizing new container
	v.resize(it - v.begin());
	return v;		
}

bool check_subset(std::vector<int> set, std::vector<int> subset)
{
	// This function check whether a input set of labels is a subset of the m_currentLabels
	return ( std::includes(set.begin(), set.end(), subset.begin(), subset.end()) );	
}



