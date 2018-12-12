/*This hpp file declares a labeled graph with different weights assigned to each label. It includes
  four data structures to represent the graph. 

  1. m_nodeNeighbors: store neighbors for each node
  2. m_edgeLabels: store the labels for each edge between two nodes
  3. m_labels: store all possible labels
  4. m_labelWeights : store the weight for each label

*/
#ifndef LABELEDGRAPH_H
#define LABELEDGRAPH_H

#include <vector>
//#include <cmath> // pow()
//#include <algorithm> // std::set_union, std::sort
//#include <bitset> // for bitwise operation
#include <map>
//#include <cassert>
#include <functional>
#include <set>



// Declaring the type of Predicate that accept two pairs and return a bool
typedef std::function<bool(std::pair<std::vector<int>, double>, 
	std::pair<std::vector<int>, double>)> Comparator;


class LabeledGraph_t
{
	// the size of the grid
	int m_row;
	int m_col;
	int m_nNodes;
	// specify neighbors (edge) and labels of the edge 
	std::vector<std::vector<int>> m_nodeNeighbors;
	std::vector<std::vector<std::vector<int>>> m_edgeLabels;
	// specify the weights for labels
	std::vector<double> m_labelWeights;
	std::vector<int> m_labels;

	std::set<std::pair<std::vector<int>, double>, Comparator> m_labelMap;
public:
	// Constructor
	LabeledGraph_t() {}
	LabeledGraph_t(int row, int col);

	// function to load a graph (manually generate a graph)
	void load_graph();

	// function to specify the neighbors for each node
	void specify_neighbors();

	// function to specify the labels for each edge
	void specify_labels();

	// function to specify all possible labels
	void load_labels();

	// function to assign weight for the labels
	void load_weights();

	// function for printing (test) purpose only
	void graph_print();

	// function to compute weight for a set of labels
	double compute_weight(std::vector<int> labels);

	// function to compute weight for a power set
	std::vector<double> compute_weights(std::vector<std::vector<int>> powerSet);

	// function to compute a powerset given a set of labels
	std::vector<std::vector<int>> cal_powerSet();

	// function to zip a and b into a map
	std::map<std::vector<int>, double> zip_combinations(
		std::vector<std::vector<int>>& a, std::vector<double>& b);

	// function to calculate labelMap
	void cal_labelMap();

	// function to print labelMap for test purpose so far
	void print_labelMap();

	// lots of getters
	std::vector<std::vector<int>> getNodeNeighbors() { return m_nodeNeighbors; }
	std::vector<std::vector<std::vector<int>>> getEdgeLabels() { return m_edgeLabels; } 
	std::vector<double> getLabelWeights() { return m_labelWeights; }

	// Destructor
	~LabeledGraph_t();
};

#endif