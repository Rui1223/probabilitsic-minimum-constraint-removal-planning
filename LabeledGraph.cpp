/*This cpp file defines a LabeledGraph class which we will use as the labeled graph in our problem 
formulation*/

#include <cstdio>
#include "LabeledGraph.hpp"

LabeledGraph_t::LabeledGraph_t(int n_nodes)
	: m_n_nodes(n_nodes)
{
	LabeledGraph_t::load_graph();
}

void LabeledGraph_t::load_graph()
{
	printf("Play tennis NOW!");
}

