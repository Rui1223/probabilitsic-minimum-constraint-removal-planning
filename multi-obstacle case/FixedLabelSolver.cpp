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
#include <cstdlib> // std::rand, std::srand
#include <iomanip>

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

// random generator function:
int myrandom1 (int i) { return std::rand()%i; }

FixedLabelSolver_t::FixedLabelSolver_t(ConnectedGraph_t &g)
{
	Timer tt;
	tt.reset();
	// problem formulation specified at the beginning of the solver
	m_lgraph = g;
	std::cout << "Time to load the graph for Fsolver: " << tt.elapsed() << " seconds\n";
	m_start = m_lgraph.getmStart();
	m_goal = m_lgraph.getmGoal();

	m_nTotallabels = m_lgraph.getnTotallabels();
	m_labelWeights = m_lgraph.getLabelWeights();

	// figure out whether start and goal are already in a region where labels are assigned (doomed)
	std::vector<int> gLabels = cal_gLabel();
	cal_labelMap(gLabels);
	//std::vector<std::pair<std::vector<int>, double>> subLabelMap = cal_subLabelMap(gLabels);

	m_k = 0;
}

void FixedLabelSolver_t::fixedLabel_search()
{
	// The search method takes advantage of the labelMap obtained from the given graph.
	// Then for each set of labels and corresponding weight in the labelMap, construct a subgraph
	// which only contains edges which carry a subset of the current set of labels being looped on. 
	// A breast-first search is performed on the subgraph to see whether the start and the goal
	// forms a connected component. If it is, return the path, then the path is optimal. Otherwise, 
	// switch to next set of labels in the labelMap (with ascending order with respect to weight).
	bool goalFound = false;

	while (!goalFound)
	{
		m_k++;
		if (m_k > m_labelMap.size()) 
		{
			std::cout << "The whole search is complete.\n"; 
			return; 
		}

		m_currentLabels = m_labelMap[m_labelMap.size() - m_k].first;
		m_currentSurvival = m_labelMap[m_labelMap.size() - m_k].second;
		//std::cout << "current set of labels: " << m_currentLabels << "\n";
		//std::cout << "currrent weight: " << m_currentWeight << "\n"; 
		//std::cout << "start the " << k << "th search\n";
		goalFound = HeuristicSearch();
		if (goalFound)
		{
			std::cout << "We have performed " << m_k << " searches.\n\n";
		}
	}

}

bool FixedLabelSolver_t::HeuristicSearch()
{
	// Let's start the current search!!
	std::vector<int> G(m_lgraph.getnNodes(), std::numeric_limits<int>::max());
	G[m_start] = 0;
	m_expanded = std::vector<bool>(m_lgraph.getnNodes(), false); // must be re-initialized
	m_open.push(new HeuristicNode_t(m_start, G[m_start], computeH(m_start), nullptr)); // open must be clean

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
		if (current->m_id == m_goal)
		{
			std::cout << "Goal is connected all the way to the start\n";
			std::cout << "current labels: " << m_currentLabels << "\n";
			std::cout << "current survivability: " << m_currentSurvival << "\n";
			back_track_path(); // construct & print your path
			return true;
		}
		//get neighbors of the current node
		std::vector<int> neighbors = m_lgraph.getNodeNeighbors(current->m_id);
		// randomly shuffle the neighbors
		std::random_shuffle( neighbors.begin(), neighbors.end(), myrandom1 );
		for (auto const &neighbor : neighbors)
		{
			// check if the node has been visited or extended before
			if (m_expanded[neighbor]) { continue; }
			// check whether the edge between current and neighbor node form a valid edge in the 
			// subgraph
			std::vector<int> EdgeLabels = m_lgraph.getEdgeLabels(current->m_id, neighbor);
			bool isSubset = check_subset(m_lgraph.getEdgeLabels(current->m_id, neighbor));
			if (isSubset)
			{
				// the neighbor is a true neighbor in the current subgraph
				// add the neighbor to open list
				if (G[current->m_id]+1 < G[neighbor])
				{
					G[neighbor] = G[current->m_id]+1;
				}
				m_open.push(new HeuristicNode_t(neighbor, G[neighbor], computeH(neighbor), current));
			}
		}
	}
	// You are reaching here because the open list is empty and goal is not found
	//std::cout << "Coundn't found the goal at the current subgraph\n";
	//std::cout << "-----------------------------------------------\n";
	// before return, free all the memory space
	for (auto &e : m_closed)
	{
		delete e;
		e = nullptr;
	}
	return false;
}

void FixedLabelSolver_t::write_solution(std::string file_dir, double t)
{
	std::ofstream file_(file_dir);
	if (file_.is_open())
	{
		// line 1: write in time & survivability
		file_ << t << " " << m_currentSurvival << "\n";

		// line 2: write in labels that the optimal solution carries
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

		// line 3: write in the path
		for (auto const &waypoint : m_path)
		{
			file_ << waypoint << " ";
		}
		file_ << "\n";

		file_.close();
	}
}

void FixedLabelSolver_t::cal_labelMap(std::vector<int> gLabels)
{
	Timer tp;
	cal_powerSet(gLabels);
	compute_survival();

	for (int kkk=0; kkk < m_survivalCombinations.size(); kkk++)
	{
		m_labelMap.push_back(std::pair<std::vector<int>, double>(m_labelCombinations[kkk], 
																	m_survivalCombinations[kkk]));
	}

	sort(m_labelMap.begin(), m_labelMap.end(), sortbysec_fixedlabel);
	std::cout << "Time to compute labelMap is: " << tp.elapsed() << " seconds\n";
}

void FixedLabelSolver_t::cal_powerSet(std::vector<int> gLabels)
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
		if (check_subset_two(labels, gLabels)) 
		{
			m_labelCombinations.push_back(labels);
		}
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

std::vector<int> FixedLabelSolver_t::cal_gLabel()
{
	std::vector<int> goal_neighbors = m_lgraph.getNodeNeighbors(m_goal);
	std::vector<std::vector<int>> goal_setsLabels;

	for (auto const &neighbor : goal_neighbors)
	{
		goal_setsLabels.push_back(m_lgraph.getEdgeLabels(m_goal, neighbor));
	}

	std::vector<int> goal_labels = goal_setsLabels[0];
	for (int kk=1; kk < goal_setsLabels.size(); kk++)
	{
		goal_labels = label_intersection(goal_labels, goal_setsLabels[kk]);
	}
	//std::cout << "start_labels: " << start_labels << "\n";
	std::cout << "goal_labels: " << goal_labels << "\n";

	// So far we have compute start_labels and goal_labels
	// We need to compute the union of start_labels & goal_labels
	//std::vector<int> sgLabels = label_union(start_labels, goal_labels);

	return goal_labels;
}

void FixedLabelSolver_t::back_track_path()
{
	// start from the goal
	HeuristicNode_t *current = m_closed[m_closed.size()-1];
	while (current->m_id != m_start)
	{
		// keep backtracking the path until you reach the start
		m_path.push_back(current->m_id);
		current = current->m_parent;
	}
	// finally put the start into the path
	m_path.push_back(current->m_id);

	std::cout << "path: " << m_path << "\n";
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


// std::vector<std::pair<std::vector<int>, double>> FixedLabelSolver_t::cal_subLabelMap(
// 																		std::vector<int> sgLabels)
// {
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


int FixedLabelSolver_t::computeH(int indx)
{
	int col = m_lgraph.getnCol();
	int indx_row = indx / col;
	int indx_col = indx % col;
	int goal_row = m_goal / col;
	int goal_col = m_goal % col;
	
	// manhattan distance as distance metric
	int h = abs(indx_row - goal_row) + abs(indx_col - goal_col);
	// int g = abs(indx_row - start_row) + abs(indx_col - start_col);
	// int f = h + g;
	// std::vector<int> fgh{f, g, h};
	return h;
}

bool FixedLabelSolver_t::check_subset(std::vector<int> labels)
{
	// This function check whether a input set of labels is a subset of the m_currentLabels
	bool isSubset = std::includes(m_currentLabels.begin(), m_currentLabels.end(),
						labels.begin(), labels.end());
	return isSubset;
}

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

bool check_subset_two(std::vector<int> set, std::vector<int> subset)
{
	// This function check whether a input set of labels is a subset of the m_currentLabels
	return ( std::includes(set.begin(), set.end(), subset.begin(), subset.end()) );	
}

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

std::ostream& operator<<(std::ostream &out, const std::vector<int> &v)
{
	out << "<"; 
	for (auto const &item : v)
	{
		out << item << " ";
	}
	out << ">";
	return out;
}




