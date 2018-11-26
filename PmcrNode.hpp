/*This hpp file declares a type of node that will be used in greedy search in probabilistic MCR*/

#include <vector>

class PmcrNode
{
	int id; // id of the node
	std::vector<int> labels;
	int label_cardinality;
	double weights; // total weights of labels
	PmcrNode *parent; // a pointer points to its parent node
};