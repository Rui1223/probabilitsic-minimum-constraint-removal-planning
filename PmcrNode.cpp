/*This cpp file provides a definition of PmcrNode used in greedy search in probabilistic MCR*/

#include "PmcrNode.hpp"
#include <iostream>
#include <cassert>

PmcrNode_t::PmcrNode_t() {}

PmcrNode_t::PmcrNode_t(int id, std::vector<int> labels, 
	PmcrNode_t *parent, double weights)
{
	assert(id >= 0);
	m_id = id;
	m_labels = labels;
	m_labelCardinality = labels.size();
	m_weights = weights;
	m_parent = parent;
	
	
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
}



