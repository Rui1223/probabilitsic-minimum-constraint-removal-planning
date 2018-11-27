/*This hpp file declares a type of node that will be used in greedy search in probabilistic MCR*/

#ifndef PMCRNODE_H
#define PMCRNODE_H

#include <vector>

class PmcrNode_t
{
	int m_id; // id of the node
	std::vector<int> m_labels;
	int m_labelCardinality;
	double m_weights; // total weights of labels
	PmcrNode_t *m_parent; // a pointer points to its parent node

public:
	// constructor
	PmcrNode_t(int id, std::vector<int> labels, PmcrNode_t *parent, 
		std::vector<double> labelWeights);
	void updateWeight(std::vector<double> labelWeights);
	void print();
	friend bool operator<(const PmcrNode_t &n1, const PmcrNode_t &n2);
};

#endif