/*This hpp file declares a connected labeled graph with different weights assigned to each label 
for different obstacles. */
/*we start with each label, expand the edges with that label based on the expected number of edge
given to that label*/
#ifndef CONNECTEDGRAPH_H
#define CONNECTEDGRAPH_H

#include <vector>
//#include <cmath> // pow()
//#include <algorithm> // std::set_union, std::sort
//#include <bitset> // for bitwise operation
#include <map>
//#include <cassert>
#include <functional>
#include <set>
#include <cstring>

// Declaring the type of Predicate that accept two pairs and return a bool
// typedef std::function<bool(std::pair<std::vector<int>, double>, 
// 	std::pair<std::vector<int>, double>)> Comparator;

class ConnectedGraph_t
{
	// the size of the grid
	int m_row;
	int m_col;
	int m_nNodes;
	int m_nEdges;

	// specify the weights for labels
	std::vector<int> m_nlabelsPerObs;
	// for each label idx, we store the obs idx it belongs to and its corresponding weight
	std::map<int, std::pair<int, double>> m_labelWeights;
	// #obstacles
	int m_nobstacles;
	int m_nTotallabels;

	// specify neighbors (edges) and labels of the edge 
	std::vector<std::vector<int>> m_nodeNeighbors;
	std::vector<std::vector<std::vector<int>>> m_edgeLabels;

	// the probability that a label is assigned to an edge
	double m_probPerLabel;
	// the density which meansures how densely the graph is labeled
	double m_labelCoverage;
	std::vector<std::vector<bool>> m_marked;
	int m_nmarked;
	int m_nExpansion;

	// all label combinations
	//std::vector<std::vector<int>> m_labelCombinations;
	//std::vector<std::pair<std::vector<int>, double>> m_labelMap;

	// multiple goals scenarios
	// since goal is now attached to the obstacle
	std::vector<int> m_goalSet;
	int m_targetObs;
	std::vector<int> m_targetPoses;
	int m_start;


public:
	// Constructor
	ConnectedGraph_t() {}
	ConnectedGraph_t(int row, int col, std::vector<int> nlabelsPerObs, double probPerLabel);

	// function to load a graph (manually generate a graph)
	void load_graph();

	// function to label the graph
	void label_graph();

	// function to get the location of a node given the index
	std::pair<int, int> getLoc(int node_idx);

	// function to check if a node is close to any of the centroid of all previous obstacles
	bool is_close(std::pair<int, int> &a, std::vector<std::pair<int, int>> &b, 
															int obs, double threshold);

	// function to return the distance of two nodes (on grid graph)
	int dist(std::pair<int, int> &a, std::pair<int, int> b);

	// function to assign weight for the labels of the same obstacle
	std::vector<double> load_weights(int nlabels);

	// function to assign weight to each label
	void assign_weight_per_label();

	// function for printing (test) purpose only
	void print_graph();

	// function to compute the survivability for a single set of labels
	double compute_survival_currentLabels(std::vector<int> labels);

	// // function to compute survivability for a power set
	// std::vector<double> compute_survival();

	// // function to compute a powerset given a set of labels
	// void cal_powerSet();

	// function to calculate labelMap
	//void cal_labelMap();

	// function to print labelMap for test purpose so far
	//void print_labelMap();

	// function to write the graph into a txt file
	void write_graph(std::string file_dir);

	// function to perform a BF-like expansion for a label
	// and if the obstacle is also a target, let it know as an input
	void BFSearch(int BF_start, int l, int obs_idx);

	// function to pick a start
	void pick_start();

	// lots of getters
	int getnCol() { return m_col; }
	int getnNodes() { return m_nNodes; }
	std::vector<int> getNodeNeighbors(int id) { return m_nodeNeighbors[id]; }
	std::vector<int> getEdgeLabels(int id1, int id2) 
														{ return m_edgeLabels[id1][id2]; } 
	std::pair<int, double> getLabelWeights(int label_idx) { return m_labelWeights[label_idx]; }
	
	int getmStart() { return m_start; }
	std::vector<int> getmGoalSet() { return m_goalSet; }
	int getmTargetObs() { return m_targetObs; }
	std::vector<int> getmTargetPoses() { return m_targetPoses; }

	// std::vector<std::pair<std::vector<int>, double>> getLabelMap()
	// {
	// 	return m_labelMap;
	// }


	// Destructor
	~ConnectedGraph_t();
};

// function to generate a random integer between min and max
int random_generate_integer(int min, int max);

// operator /= overloading used to normalize a vector of doubles
void operator/=(std::vector<double> &v, double d);

#endif