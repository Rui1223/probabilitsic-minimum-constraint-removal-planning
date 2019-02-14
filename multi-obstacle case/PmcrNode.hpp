/*This hpp file declares a type of node that will be used in greedy search in probabilistic MCR*/

#ifndef PMCRNODE_H
#define PMCRNODE_H

#include <vector>
#include <memory> // for std::shared_ptr


class PmcrNode_t
{
	int m_id; // id of the node
	int m_H; // Heuristic information of the node
	std::vector<int> m_labels;
	int m_labelCardinality;
	double m_weight; // total weight of labels
	PmcrNode_t *m_parent; // a pointer points to its parent node

public:
	// constructor
	PmcrNode_t();
	PmcrNode_t(int id, int H, std::vector<int> labels, PmcrNode_t *parent, 
		double weight);
	void print();

	/*getter*/
	int getID() { return m_id; }
	int getH() const { return m_H; } 
	std::vector<int> getLabels() { return m_labels; } 
	int getCardinality() { return m_labelCardinality; } 
	double getWeight() const { return m_weight; } 
	PmcrNode_t* getParent() { return m_parent;}

};

#endif