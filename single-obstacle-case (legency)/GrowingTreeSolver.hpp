/* This hpp files declare a solver used to solve weighted MCR problem under the assumption
1. The edge sets are connected
2. The number of labels are fixed (not sure)
*/

#ifndef GROWINGTREESOLVER_H
#define GROWINGTREESOLVER_H

#include <cstring>
#include <queue>
#include <vector>

#include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
#include "ConnectedNonOverlapGraph.hpp"
#include "ToyGraph.hpp"

#endif

struct TreeNode_t
{
	int m_id;
	TreeNode_t *m_parent;
	std::vector<int> m_labels;
	double m_weight; // total weight of labels
	TreeNode_t() {}
	TreeNode_t(int id, TreeNode_t *parent, std::vector<int> labels, double weight)
		: m_id(id), m_parent(parent), m_labels(labels), m_weight(weight) {}
};

class GrowingTreeSolver_t
{
	ConnectedGraph_t m_lgraph;
	int m_start;
	int m_goal;
	std::vector<int> m_currentLabels;
	double m_currentWeight;

	std::queue<TreeNode_t*> m_open;
	std::vector<TreeNode_t*> m_closed;
	std::queue<TreeNode_t*> m_frontier;
	std::vector<std::vector<std::vector<int>>> m_labelsHistory;
	std::vector<int> m_path;

public:
	// constructor
	GrowingTreeSolver_t() {}
	GrowingTreeSolver_t(ConnectedGraph_t &g, int start, int goal);

	// The main function to search like a growing tree
	void GrowingTreeSearch();

	// The function to back track the path
	void back_track_path();

	// The function to print the path
	void print_path();

	// The function to print the closedList for testing purposes
	// void print_closedList()

	// The function to calculate the union of two labels
	std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2);
	// The function to check whether a input set of labels is a subset of the m_currentLabel
	bool check_subset(std::vector<int> labels);

	// getters
	double getCurrentWeight() { return m_currentWeight; }
	std::vector<int> getCurrentLabels() { return m_currentLabels; }

	// The function to write in the solution
	void write_solution(std::string file_dir, double t);

	~GrowingTreeSolver_t();

};
