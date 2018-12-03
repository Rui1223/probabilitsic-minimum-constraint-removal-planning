/*This hpp file declares a labeled graph with different weights assigned to each label. It includes
  three data structures to represent the graph. 

  1. m_nodeNeighbors: store neighbors for each node
  2. m_edgeLabels: store the labels for each edge between two nodes
  3. m_labelWeights : store the weight for each label

*/
#ifndef LABELEDGRAPH_H
#define LABELEDGRAPH_H

#include <vector>

class LabeledGraph_t
{
	int m_gridSize; // the size of the grid
	std::vector<std::vector<int>> m_nodeNeighbors;
	std::vector<std::vector<std::vector<int>>> m_edgeLabels;
	std::vector<double> m_labelWeights;
public:
	// Constructor
	LabeledGraph_t() {}
	LabeledGraph_t(int n_nodes);

	// function to load a graph (manually generate a graph)
	void load_graph();

	// function to specify the neighbors for each node
	void specify_neighbors();

	// function to specify the labels for each edge
	void specify_labels();

	// function to assign weight for the labels
	void load_weights();

	// function for printing (test) purpose only
	void graph_print();

	// lots of getters
	std::vector<std::vector<int>> getNodeNeighbors() { return m_nodeNeighbors; }
	std::vector<std::vector<std::vector<int>>> getEdgeLabels() { return m_edgeLabels; } 
	std::vector<double> getLabelWeights() { return m_labelWeights; }

	// Destructor
	~LabeledGraph_t();
};

#endif