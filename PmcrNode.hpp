/*This hpp file declares a type of node that will be used in greedy search in probabilistic MCR*/

#ifndef PMCRNODE_H
#define PMCRNODE_H

#include <vector>
#include <memory> // for std::shared_ptr


class PmcrNode_t
{
	int m_id; // id of the node
	std::vector<int> m_labels;
	int m_labelCardinality;
	double m_weights; // total weights of labels
	PmcrNode_t *m_parent; // a pointer points to its parent node

public:
	// constructor
	PmcrNode_t();
	PmcrNode_t(int id, std::vector<int> labels, PmcrNode_t *parent, 
		double weights);
	void print();

	/*getter*/
	int getID() { return m_id; } 
	std::vector<int> getLabels() { return m_labels; } 
	int getCardinality() { return m_labelCardinality; } 
	double getWeights() const { return m_weights; } 
	PmcrNode_t* getParent() { return m_parent;}

	//friend bool operator<(const std::shared_ptr<PmcrNode_t> n1, const std::shared_ptr<PmcrNode_t> n2);


};

#endif