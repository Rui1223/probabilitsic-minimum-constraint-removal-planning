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
	// problem formulation specified at the beginning of the solver
	m_lgraph = g;
	std::cout << "Time to load the graph for Fsolver: " << tt.elapsed() << " seconds\n\n";
	m_start = m_lgraph.getmStart();
	m_goalSet = m_lgraph.getmGoalSet();
	m_targetObs = m_lgraph.getmTargetObs();
	m_targetPoses = m_lgraph.getmTargetPoses();

	//essential elements for fixedLabel search
	m_paths = std::vector<std::vector<int>>();

	m_nTotallabels = m_lgraph.getnTotallabels();
	m_labelWeights = m_lgraph.getLabelWeights();

	// construct labelMap here.
	cal_labelMap();
	// print_labelMap();

	// Initialize 3 key variables we will be using
	m_highestSuccess = 0.0;
	m_lowestReachability = 0.0;

	m_k = 0;

	//print_goalSet();
}

void FixedLabelSolver_t::fixedLabel_search()
{
	// Then for each set of labels and corresponding weight in the labelMap, construct a subgraph
	// which only contains edges which carry a subset of the current set of labels being looped on. 
	// An A* search is performed on the subgraph to see whether we can reach any one of the goal 
	// from the start. If it is, return the path to be the optimal so far. Otherwise
	// switch to next set of labels in the labelMap (with ascending order with respect to 
	// survivability).

	while (!m_goalSet.empty())
	{
		m_k++;
		if (m_k > m_labelMap.size()) 
		{
			std::cout << "The whole search is complete.\n"; 
			return; 
		}
		//std::cout << "start the " << k << "th search...\n";
		// get current labels and corresponding survivability
		m_currentLabels = m_labelMap[m_labelMap.size() - m_k].first;
		m_currentSurvival = m_labelMap[m_labelMap.size() - m_k].second;
		HeuristicSearch();
	}
	std::cout << "We have performed " << m_k << " searches.\n";
	std::cout << "No goals left in the goalSet, done.\n";
}

void FixedLabelSolver_t::HeuristicSearch()
{
	m_MaxSurvival = m_currentSurvival; // won't change in a single HeuristicSearch() call
	m_lowestReachability = m_highestSuccess*1.0 / m_MaxSurvival;
	// Before the search, maybe we can already prune some goals
	prune_goalSet();
	// check if the goalSet is empty, if it is, we are done.
	if (m_goalSet.empty()) 
	{ 
		return; 
	}

	// We are reaching here since there are goals remaining for us to search
	// Let's start the current search!!
	m_expanded = std::vector<bool>(m_lgraph.getnNodes(), false); // must be re-initialized
	m_open.push(new HeuristicNode_t(m_start, computeFGH(m_start), nullptr)); // open must be clean

	int goal_idx;
	std::vector<int>::iterator it;

	while (!m_open.empty())
	{
		HeuristicNode_t *current = m_open.top();
		m_open.pop();
		// Now check if the current node has been expanded
		if (m_expanded[current->m_id] == true)
		{
			// Has been expanded, no need to put it into the closed list
			delete current;
			continue;
		}
		m_closed.push_back(current);
		m_expanded[current->m_id] = true;

		// look at each neighbor of the current node
		std::vector<int> neighbors = m_lgraph.getNodeNeighbors(current->m_id);
		for (auto const &neighbor : neighbors)
		{
			if (m_expanded[neighbor] == true) 
			{ 
				continue; 
			}
			// check whether the edge between current and neighbor node form a valid edge in the 
			// subgraph
			std::vector<int> EdgeLabels = m_lgraph.getEdgeLabels(current->m_id, neighbor);
			bool isSubset = check_subset(m_lgraph.getEdgeLabels(current->m_id, neighbor));
			if (isSubset)
			{
				// the neighbor is a true neighbor in the current subgraph
				// add the neighbor to open list
				m_open.push(new HeuristicNode_t(neighbor, computeFGH(neighbor), current));
			} 
		}
		// Now check whether we reach a goal
		it = std::find(m_goalSet.begin(), m_goalSet.end(), current->m_id);
		// reach the goal
		if ( it != m_goalSet.end() )
		{
			std::cout << "Encounter a goal\n";
			goal_idx = std::distance(m_goalSet.begin(), it);
			// print the goal & its corresponding target pose
			std::cout << "Goal: " << current->m_id << "\n";
			std::cout << "Target Pose: " << m_targetPoses[goal_idx] << "\n";
			// check if the labels the path from start to current carries contains the label of the
			// goal you are reaching. If it is, it fails. (Since you cannot reach the goal without
			// colliding it.)
			if ( std::find(m_currentLabels.begin(), m_currentLabels.end(), m_targetPoses[goal_idx]) 
																			!= m_currentLabels.end() )
			{
				// directly prune the goal
				std::cout << "has to reach the goal while colliding with that goal pose,prune.\n\n";
				m_goalSet.erase(it);
				m_targetPoses.erase(m_targetPoses.begin()+goal_idx);
				//print_goalSet();
				// check if the goalSet is empty, if it is, we are done.
				if (m_goalSet.empty()) { return; }
				continue;
			}
			// Otherwise it is a valid goal and let's back track of the path
			std::cout << "It's a valid goal\n";
			// update m_highestSuccess & m_lowestReachability					
			m_highestSuccess = 
				m_MaxSurvival * m_lgraph.getLabelWeights(m_targetPoses[goal_idx]).second * 1.0;
			m_lowestReachability = m_highestSuccess*1.0 / m_MaxSurvival;	
			// print the success rate, survivability & labels for the found path
			std::cout << "Success rate of the path: " << m_highestSuccess << "\n";
			std::cout << "Survivability of the path: " << m_MaxSurvival << "\n";
			std::cout << "Labels of the path: " << "<";
			for (auto const &e : m_currentLabels)
			{
				std::cout << e << " ";
			}
			std::cout << ">\n";

			// should return a path here
			back_track_path();
			std::cout << "This is " << m_k << "th search...\n";


			m_optimalGoal = m_goalSet[goal_idx];
			m_optimalPose = m_targetPoses[goal_idx];
			m_optimalSurvival = m_currentSurvival;
			m_optimalLabels = m_currentLabels;

			// delete that goal from the goalSet
			m_goalSet.erase(it);
			m_targetPoses.erase(m_targetPoses.begin()+goal_idx);
			//print_goalSet();
			// check if the goalSet is empty, if it is, we are done.
			if (m_goalSet.empty()) { return; }
			// Prune the goal set based on the latest computed m_lowestReachability
			prune_goalSet();
			std::cout << "-----------------------------------------\n\n";
			if (m_goalSet.empty()) { return; }		

		}

	}
	// You are reaching here since the open list is empty and you are not exhausting all the goals
	// (since if you are, the goalSet will be empty and the HeuristicSearch() will terminated and return)

	// need to clean closed list for next search iteration
	for (auto &e : m_closed)
	{
		delete e;
		e = nullptr;
	}	

}

std::vector<int> FixedLabelSolver_t::computeFGH(int indx)
{
	int col = m_lgraph.getnCol();
	int indx_row = indx / col;
	int indx_col = indx % col;
	int start_row = m_start / col;
	int start_col = m_start % col;
	int g = abs(indx_row - start_row) + abs(indx_col - start_col);

	// manhattan distance as distance metric	
	int goal_row;
	int goal_col;
	int min_H = std::numeric_limits<int>::max();
	int temp_h;
	// the heuristic value now is the minimum manhattan distance among all goals
	for (auto const &g : m_goalSet)
	{
		goal_row = g / col;
		goal_col = g % col;
		temp_h = abs(indx_row - goal_row) + abs(indx_col - goal_col);
		if (temp_h < min_H) { min_H = temp_h; }
	}
	int f = min_H + g;
	std::vector<int> fgh{f, g, min_H};
	return fgh;
}

void FixedLabelSolver_t::prune_goalSet()
{
	int deletions = 0;
	for (int gg=0; gg < (m_goalSet.size()+deletions); gg++)
	{
		if (m_lgraph.getLabelWeights(m_targetPoses[gg-deletions]).second < m_lowestReachability)
		{
			// std::cout << "m_lowestReachability: " << m_lowestReachability << "\n";
			// std::cout << "prune goal: " << m_goalSet[gg - deletions] 
			// 			<< " for pose: " << m_targetPoses[gg - deletions] << "\n";
			m_goalSet.erase(m_goalSet.begin() + gg - deletions);
			m_targetPoses.erase(m_targetPoses.begin() + gg - deletions);
			deletions++;
			//print_goalSet();
		}
	}
}

void FixedLabelSolver_t::print_goalSet()
{
	std::cout << "current m_goalSet: ";
	for (auto const &gs : m_goalSet)
	{
		std::cout << gs << " "; 
	}
	std::cout << "\n\n";
}

bool FixedLabelSolver_t::check_subset(std::vector<int> labels)
{
	// This function check whether a input set of labels is a subset of the m_currentLabels
	bool isSubset = std::includes(m_currentLabels.begin(), m_currentLabels.end(),
						labels.begin(), labels.end());
	return isSubset;
}

void FixedLabelSolver_t::back_track_path()
{
	std::vector<int> path_candidate;
	// start from the goal
	HeuristicNode_t *current = m_closed[m_closed.size()-1];
	while (current->m_id != m_start)
	{
		// keep backtracking the path until you reach the start
		path_candidate.push_back(current->m_id);
		current = current->m_parent;
	}
	// finally put the start into the path
	path_candidate.push_back(current->m_id);

	// print the current path
	for (auto const &waypoint : path_candidate)
	{
		std::cout << waypoint << " ";
	}
	std::cout << "\n";

	m_paths.push_back(path_candidate);
}



void FixedLabelSolver_t::write_solution(std::string file_dir, double t)
{

	std::ofstream file_(file_dir);
	if (file_.is_open())
	{
		file_ << t << " " << m_highestSuccess << " " << m_optimalSurvival
				<< " " << m_optimalGoal << " " << m_optimalPose << " " << m_paths.size() << "\n";

		if (!m_optimalLabels.empty())
		{
			int pp = 0;
			while (pp < m_optimalLabels.size()-1)
			{
				file_ << m_optimalLabels[pp] << ",";
				pp++;
			}
			file_ << m_optimalLabels[pp];
		}
		file_ << "\n";

		// write in all paths
		for (auto const &p : m_paths)
		{
			for (auto const &waypoint : p)
			{
				file_ << waypoint << " ";
			}
			file_ << "\n";
		}
		file_ << "\n";
		file_.close();
	}


}

void FixedLabelSolver_t::cal_labelMap()
{
	cal_powerSet();
	compute_survival();

	for (int kkk=0; kkk < m_survivalCombinations.size(); kkk++)
	{
		m_labelMap.push_back(std::pair<std::vector<int>, double>(m_labelCombinations[kkk], 
																	m_survivalCombinations[kkk]));
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

void FixedLabelSolver_t::compute_survival()
{
	for (auto const &set : m_labelCombinations)
	{
		double survival = m_lgraph.compute_survival_currentLabels(set);
		m_survivalCombinations.push_back(survival);
	}

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

// std::vector<int> label_intersection(std::vector<int> v1, std::vector<int> v2)
// {
// 	std::vector<int> s;
// 	std::sort(v1.begin(), v1.end());
// 	std::sort(v2.begin(), v2.end());
// 	std::set_intersection(v1.begin(),v1.end(),v2.begin(),v2.end(), back_inserter(s));
// 	return s;
// }

// std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2)
// {
// 	// sort the sets first before applying union operation
// 	std::sort(s1.begin(), s1.end());
// 	std::sort(s2.begin(), s2.end());

// 	// Declaring resultant vector for union
// 	std::vector<int> v(s1.size()+s2.size());
// 	// using function set_union() to compute union of 2
// 	// containers v1 and v2 and store result in v
// 	auto it = std::set_union(s1.begin(), s1.end(), s2.begin(), s2.end(), v.begin());

// 	// resizing new container
// 	v.resize(it - v.begin());
// 	return v;		
// }

// bool check_subset(std::vector<int> set, std::vector<int> subset)
// {
// 	// This function check whether a input set of labels is a subset of the m_currentLabels
// 	return ( std::includes(set.begin(), set.end(), subset.begin(), subset.end()) );	
// }

FixedLabelSolver_t::~FixedLabelSolver_t()
{
	while (!m_open.empty())
	{
		HeuristicNode_t* a1 = m_open.top();
		delete a1;
		m_open.pop();
	}
	for (auto &e : m_closed) { delete e; }
}



