#include <vector>
#include <iostream>
#include <cassert>

#include "PmcrNode.hpp"
#include "LabeledGraph.hpp"
#include "PmcrGreedySolver.hpp"

PmcrGreedySolver_t::PmcrGreedySolver_t(LabeledGraph_t g, int start, int goal)
{
	m_lgraph = g;
	assert(start >=0);
	assert(goal >=0);
	m_start = start;
	m_goal = goal;
	m_open.push(PmcrNode_t(start, {0}, nullptr, {0.0, 0.1, 0.6, 0.3}));
}