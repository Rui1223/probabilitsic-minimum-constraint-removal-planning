/*This hpp file declares a labeled graph. It includes two data structures to represent the graph. 
One is to store the neighbor of each node, and the other is to store labels between two nodes */

#include <vector>

class LabeledGraph_t
{
	int m_n_nodes; // the number of nodes
	std::vector<std::vector<int>> m_nodeNeigbors;
	std::vector<std::vector<std::vector<int>>> m_edgeLabels;
public:
	// Constructor
	LabeledGraph_t(int n_nodes);

	// function to load a graph
	void load_graph();
};