/*This hpp file declares a type of node that will be used in greedy search in probabilistic MCR*/

#ifndef PMCRNODE_H
#define PMCRNODE_H

#include <vector>
#include <memory> // for std::shared_ptr


class PmcrNode_t
{
	int m_id; // id of the node
	std::vector<int> m_FGH; // f,g,h value of the node
	std::vector<int> m_labels;
	int m_labelCardinality;
	double m_survival; // total survivability of labels
	PmcrNode_t *m_parent; // a pointer points to its parent node

public:
	// constructor
	PmcrNode_t();
	PmcrNode_t(int id, std::vector<int> FGH, std::vector<int> labels, PmcrNode_t *parent, 
		double survival);
	void print();

	/*getter*/
	int getID() { return m_id; }
	int getF() const { return m_FGH[0]; }
	int getG() const { return m_FGH[1]; }
	int getH() const { return m_FGH[2]; }
	std::vector<int> getLabels() { return m_labels; } 
	int getCardinality() { return m_labelCardinality; } 
	double getSurvival() const { return m_survival; } 
	PmcrNode_t* getParent() { return m_parent;}

};

#endif