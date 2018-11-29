/*This cpp file provides a definition of PmcrNode used in greedy search in probabilistic MCR*/

#include "PmcrNode.hpp"
#include <iostream>
#include <cassert>

PmcrNode_t::PmcrNode_t(int id, std::vector<int> labels, 
	PmcrNode_t *parent, std::vector<double> labelWeights)
{
	assert(id >= 0);
	m_id = id;
	m_labels = labels;
	m_parent = parent;
	m_labelCardinality = labels.size();
	updateWeight(labelWeights);
}

void PmcrNode_t::updateWeight(std::vector<double> labelWeights)
{
	m_weights = 0.0;
	for (auto const &label : m_labels)
	{
		m_weights += labelWeights[label]; 
	}
}

void PmcrNode_t::print()
{
	std::cout << m_id << " ";
	for (auto const &label : m_labels)
	{
		std::cout << label << ",";
	}
	std::cout << " ";
	std::cout << m_labelCardinality << " ";
	std::cout << m_weights << " ";
	std::cout << m_parent << "\n";
}


bool operator<(const PmcrNode_t &n1, const PmcrNode_t &n2)
{
	// Here is an operator overloading on class PmcrNode
	// to compare the total weights of nodes
	// will be used in priority queue
	// those nodes who have less weights will be
	// on top of the priority queue
	return n1.m_weights > n2.m_weights;
}

