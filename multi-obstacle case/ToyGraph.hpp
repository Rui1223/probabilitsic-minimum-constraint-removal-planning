/*This hpp file declares a toy graph in a multi-obstacle scenario to test the extensible algorithms. 
*/
#ifndef TOYGRAPH_H
#define TOYGRAPH_H

#include <vector>
//#include <cmath> // pow()
//#include <algorithm> // std::set_union, std::sort
//#include <bitset> // for bitwise operation
#include <map>
//#include <cassert>
#include <functional>
#include <set>
#include <cstring> // for std::string & std::to_string

// Declaring the type of Predicate that accept two pairs and return a bool
// typedef std::function<bool(std::pair<std::vector<int>, double>, 
// 	std::pair<std::vector<int>, double>)> Comparator;

class ToyGraph_t
{
	// the size of the grid
	int m_row;
	int m_col;
	int m_nNodes;
	int m_nEdges;

	// specify the weights for labels
	std::vector<int> m_nlabelsPerObs;
	// for each label idx, we store the obs idx it belongs and its corresponding weight
	std::map<int, std::pair<int, double>> m_labelWeights;
	// #obstacles
	int m_nobstacles;
	int m_nTotallabels;

	// specify neighbors (edges) and labels of the edge 
	std::vector<std::vector<int>> m_nodeNeighbors;
	std::vector<std::vector<std::vector<int>>> m_edgeLabels;

	// all label combinations
	std::vector<std::vector<int>> m_labelCombinations;
	//std::vector<std::pair<std::vector<int>, double>, Comparator> m_labelMap;
	std::vector<std::pair<std::vector<int>, double>> m_labelMap;


public:
	// Constructor
	ToyGraph_t() {}
	ToyGraph_t(int row, int col, std::vector<int> nlabelsPerObs);

	// function to load a graph (manually generate a graph)
	void load_graph();

	// function to assign weight to each label
	void assign_weight_per_label();

	// function for printing (test) purpose only
	void print_graph();

	// function to compute the survivability for a single set of labels
	double compute_survival_currentLabels(std::vector<int> labels);

	// function to compute survivability for a power set
	std::vector<double> compute_survival();

	// function to compute a powerset given a set of labels
	void cal_powerSet();

	// function to calculate labelMap
	void cal_labelMap();

	// function to print labelMap for test purpose so far
	void print_labelMap();

	// // function to write the graph into a txt file
	// void write_graph(std::string file_dir);

	// lots of getters
	int getnCol() { return m_col; }
	int getnNodes() { return m_nNodes; }
	std::vector<int> getNodeNeighbors(int id) { return m_nodeNeighbors[id]; }
	std::vector<int> getEdgeLabels(int id1, int id2) 
														{ return m_edgeLabels[id1][id2]; }
	std::pair<int, double> getLabelWeights(int label_idx) { return m_labelWeights[label_idx]; }
	std::vector<std::pair<std::vector<int>, double>> getLabelMap()
	{
		return m_labelMap;
	}

	// Destructor
	~ToyGraph_t();
};

// function to generate a random integer between min and max
int random_generate_integer(int min, int max);

// operator /= overloading used to normalize a vector of doubles
void operator/=(std::vector<double> &v, double d);

#endif