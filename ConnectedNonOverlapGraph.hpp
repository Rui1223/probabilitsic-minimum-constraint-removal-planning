/*This hpp file declares a connected and non-overlapping labeled graph with different weights 
assigned to each label. we start with each label, expand the edges with that label 
based on the expected number of edge given to that label*/
#ifndef CONNECTEDNONOERLAPGRAPH_H
#define CONNECTEDNONOERLAPGRAPH_H

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


class ConnectedNonOverlapGraph_t
{
	// the size of the grid
	int m_row;
	int m_col;
	int m_nNodes;
	int m_nEdges;
	// specify neighbors (edge) and labels of the edge 
	std::vector<std::vector<int>> m_nodeNeighbors;
	std::vector<std::vector<std::vector<int>>> m_edgeLabels;
	std::vector<std::vector<int>> m_marked;
	
	// specify the weights for labels
	int m_nlabels;
	std::vector<double> m_labelWeights;
	std::vector<int> m_labels;

	// all label combinations
	std::vector<std::vector<int>> m_labelCombinations;
	std::set<std::pair<std::vector<int>, double>, Comparator> m_labelMap;

	// the percent of edges that are expected to have at least a label
	double m_percentLabelEdge;
	// the probability that a label is assigned to an edge
	double m_prob;
public:
	// Constructor
	ConnectedNonOverlapGraph_t() {}
	ConnectedNonOverlapGraph_t(int row, int col, int n_labels, double percentLabelEdge);

	// function to load a graph (manually generate a graph)
	void load_graph();

	// function to label the graph
	void label_graph();

	// function to specify all possible labels
	void load_labels();

	// function to assign weight for the labels
	void load_weights();

	// function for printing (test) purpose only
	void print_graph();

	// function to compute weight for a set of labels
	double compute_weight(std::vector<int> labels);

	// function to compute weight for a power set
	std::vector<double> compute_weights();

	// function to compute a powerset given a set of labels
	void cal_powerSet();

	// function to zip labelCombination and another vector b into a map
	std::map<std::vector<int>, double> zip_combinations(std::vector<double>& b);

	// function to calculate labelMap
	void cal_labelMap();

	// function to print labelMap for test purpose so far
	void print_labelMap();

	// function to write the graph into a txt file
	void write_graph(int n);

	// function to perform a BF-like expansion for a label
	void BFSearch(int BF_start, int n_expansion, int l);

	// lots of getters
	int getnCol() { return m_col; }
	int getnNodes() { return m_nNodes; }
	std::vector<std::vector<int>> getNodeNeighbors() { return m_nodeNeighbors; }
	std::vector<std::vector<std::vector<int>>> getEdgeLabels() { return m_edgeLabels; } 
	std::vector<double> getLabelWeights() { return m_labelWeights; }
	std::vector<int> getLabels() { return m_labels; }
	std::set<std::pair<std::vector<int>, double>, Comparator> getLabelMap()
	{
		return m_labelMap;
	}

	// Destructor
	~ConnectedNonOverlapGraph_t();
};

// function to generate a random integer between min and max
int random_generate_integer(int min, int max);

// operator /= overloading used to normalize a vector of doubles
void operator/=(std::vector<double> &v, double d);

#endif