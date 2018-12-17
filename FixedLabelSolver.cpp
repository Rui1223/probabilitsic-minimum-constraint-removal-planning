/*This cpp files defines a solver used to solve weighted MCR problem under the assumption
  that each obstacle has fixed number of potential pose (equivalently, the number of labels
  is fixed in a MinLP problem)*/
/*full implementation*/

#include <queue>
#include <vector>
#include <algorithm>
#include <iostream>

#include "FixedLabelSolver.hpp"
#include "LabeledGraph.hpp"

FixedLabelSolver_t::FixedLabelSolver_t(LabeledGraph_t g, int start, int goal)
{
	// problem formulation specified at the beginning of the solver
	m_lgraph = g;
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
	int k = 0;
	for (auto const &pair : m_lgraph.getLabelMap())
	{
		m_currentLabels = pair.first;
		m_currentWeight = pair.second;
		std::cout << "current set of labels: " << m_currentLabels << "\n";
		std::cout << "currrent weight: " << m_currentWeight << "\n"; 
		std::cout << "start the " << k << "th search\n";
		bool goalFound = BFSearch();
		if (goalFound)
		{
			print_path();
			return;
		}
		k++;
		// otherwise switch to next set of labels in the labelMap (next pair)
	}

}

bool FixedLabelSolver_t::check_subset(std::vector<int> labels)
{
	// This function check whether a input set of labels is a subset of the m_currentLabels
	return ( std::includes(m_currentLabels.begin(), m_currentLabels.end(),
						labels.begin(), labels.end()) );
}

void FixedLabelSolver_t::backtrackPath(std::vector<int> &parents)
{
	int temp = m_goal;
	while (temp != m_start)
	{
		m_path.push_back(temp);
		temp = parents[temp];
	}
	m_path.push_back(m_start);
}

bool FixedLabelSolver_t::BFSearch()
{
	// BFSearch() performs a Breadth-First search with a label checking condition, which decides
	// whether a newly propogated node should be added into the queue
 	// you need two data structure to perform BFSearch
	// 1. a regular queue
	// 2. a parent list for backtracking path purpose
	// 3. a list which keep track of visited status of each node
	std::queue<int> q;
	std::vector<int> parents(m_lgraph.getnNodes(), -1); // initially all 
												//the values are -1 (means no parents)
	std::vector<bool> visited(m_lgraph.getnNodes(), false);
	q.push(m_start);
	visited[m_start] = true;


	while (!q.empty())
	{
		int current = q.front(); // get the top item of the queue
		q.pop();

		//get neighbors of the current node
		std::vector<int> neighbors = m_lgraph.getNodeNeighbors()[current];
		for (auto const &neighbor : neighbors)
		{
			if (visited[neighbor]) { continue; }
			// check if the node has been visited or extended before
			// check whether the edge between current and neighbor node form a valid edge in the 
			// subgraph
			bool isSubset = check_subset(m_lgraph.getEdgeLabels()[current][neighbor]);
			if (isSubset)
			{
				// the neighbor is a true neighbor in the current subgraph
				parents[neighbor] = current;
				// add the neighbor to the queue
				q.push(neighbor);
				visited[neighbor] = true;
				// Additional step: IMPORTANT since it may get rid of unnecessary search
				// Once goal appears, claims that goal is found and terminate
				if (neighbor == m_goal)
				{
					std::cout << "Goal is connected all the way to the start\n";
					backtrackPath(parents); // construct your m_path
					return true;
				}
			} 
		}

	}
	// You are reaching here because the queue is empty and goal is not found
	std::cout << "Coundn't found the goal at the current subgraph\n";
	std::cout << "-----------------------------------------------\n"; 
	return false;

}

void FixedLabelSolver_t::print_path()
{
	std::cout << m_path << "\n";
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