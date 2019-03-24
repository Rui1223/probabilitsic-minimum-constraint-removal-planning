/*This hpp file declares a connected labeled graph with different weights assigned to each label 
for different obstacles. */
/*we start with each label, expand the edges with that label based on the expected number of edges
given to that label*/
#ifndef CONNECTEDGRAPH_H
#define CONNECTEDGRAPH_H

#include <vector>
#include <cmath> // pow()
#include <algorithm> // std::set_union, std::sort
#include <bitset> // for bitwise operation
#include <map>
#include <cassert>
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

	// specify the number of labels for each obstacle
	std::vector<int> m_nlabelsPerObs;
	// for each label idx, we store the obs idx it belongs to and its corresponding weight
	std::map<int, std::pair<int, double>> m_labelWeights;
	// #obstacles
	int m_nobstacles;
	int m_nTotallabels;

	// specify neighbors (edges) and labels, and edge cost of the graph
	std::vector<std::vector<int>> m_nodeNeighbors;
	std::vector<std::vector<std::vector<int>>> m_edgeLabels;
	std::vector<std::vector<int>> m_edgeCosts;

	// the density which meansures how densely the graph is labeled
	std::vector<std::vector<bool>> m_marked;
	int m_nmarked;
	int m_nExpansion;

	// all label combinations
	//std::vector<std::vector<int>> m_labelCombinations;
	//std::vector<std::pair<std::vector<int>, double>> m_labelMap;

	// multiple goals scenarios
	// since goal is now attached to the obstacle
	// But would like to make sure the m_start is collision free
	int m_start;
	std::vector<int> m_goalSet;
	int m_targetObs;
	std::vector<int> m_targetPoses;

	// variables used for groundtruth (and execution & replanning)
	std::vector<bool> m_truePoses;
	std::vector<int> m_trueObs;
	int m_trueTarget;
	int m_trueGoal;

	double m_obsDistrVar;
	double m_entropy;

public:
	// Constructor
	ConnectedGraph_t() {}
	ConnectedGraph_t(int row, int col, std::vector<int> nlabelsPerObs, double obsDistrVar);

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

	// function to write the graph into a txt file
	void write_graph(std::string file_dir);

	// function to perform a BF-like expansion for a label
	// and if the obstacle is also a target, let it know as an input
	void BFSearch(int BF_start, int l, int obs_idx);

	// function to pick a start
	void pick_start();

	// groundTruth generation
	void generate_groundTruth(std::string groundTruth_dir);
	void write_groundTruth(std::string groundTruth_dir);

	// lots of getters
	int getnCol() { return m_col; }
	int getnNodes() { return m_nNodes; }
	int getnTotallabels() { return m_nTotallabels; }
	int getnObstacles() { return m_nobstacles; }
	int getnLabelsPerObs() { return m_nlabelsPerObs[0]; }
	std::vector<int> getNodeNeighbors(int id) { return m_nodeNeighbors[id]; }
	std::vector<int> getEdgeLabels(int id1, int id2) 
														{ return m_edgeLabels[id1][id2]; }
	int getEdgeCost(int id1, int id2) { return m_edgeCosts[id1][id2]; }
	std::map<int, std::pair<int, double>> getLabelWeights() { return m_labelWeights; }
	std::pair<int, double> getLabelWeights(int label_idx) { return m_labelWeights[label_idx]; }
	double getEntropy() { return m_entropy; }
	
	int getmStart() { return m_start; }
	std::vector<int> getmGoalSet() { return m_goalSet; }
	int getmTargetObs() { return m_targetObs; }
	std::vector<int> getmTargetPoses() { return m_targetPoses; }

	bool getTruePoses(int indx) { return m_truePoses[indx]; }
	int getTrueGoal() { return m_trueGoal; }

	// Destructor
	~ConnectedGraph_t();
};

// function to generate a random integer between min and max
int random_generate_integer(int min, int max);

// operator /= overloading used to normalize a vector of doubles
void operator/=(std::vector<double> &v, double d);

#endif