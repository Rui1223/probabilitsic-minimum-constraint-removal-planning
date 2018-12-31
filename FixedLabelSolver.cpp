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
#include <string> // std::string, std::to_string

#include "HeuristicSearchSolver.hpp"
#include "FixedLabelSolver.hpp"
#include "LabeledGraph.hpp"

FixedLabelSolver_t::FixedLabelSolver_t(ConnectedGraph_t &g, int start, int goal)
{
	// problem formulation specified at the beginning of the solver
	m_lgraph = g;
	assert(start >=0);
	assert(goal >=0);
	m_start = start;
	m_goal = goal;
}

void FixedLabelSolver_t::fixedLabel_search()
{
	// The search method takes advantage of the labelMap obtained from the given graph.
	// Then for each set of labels and corresponding weight in the labelMap, construct a subgraph
	// which only contains edges which carry a subset of the current set of labels being looped on. 
	// A breast-first search is performed on the subgraph to see whether the start and the goal
	// forms a connected component. If it is, return the path, then the path is optimal. Otherwise, 
	// switch to next set of labels in the labelMap (with ascending order with respect to weight).
	int k = 1;
	for (auto const &pair : m_lgraph.getLabelMap())
	{
		m_currentLabels = pair.first;
		m_currentWeight = pair.second;
		std::cout << "current set of labels: " << m_currentLabels << "\n";
		std::cout << "currrent weight: " << m_currentWeight << "\n"; 
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
	HeuristicSearchSolver_t heuristic_search_solver(m_lgraph, m_start, m_goal, 
													m_currentLabels, m_currentWeight);
	bool goalFound = heuristic_search_solver.Heuristic_search();
	if (goalFound)
	{
		m_path = heuristic_search_solver.getPath();
	}
	return goalFound;
}

void FixedLabelSolver_t::write_solution(std::string file_dir)
{
	std::ofstream file_(file_dir);
	if (file_.is_open())
	{
		file_ << m_start << " " << m_goal << "\n";
		for (auto const &waypoint : m_path)
		{
			file_ << waypoint << " ";
		}
		file_ << "\n";
		// write the label and weight for the solution
		file_ << m_currentWeight << "\n";
		
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




