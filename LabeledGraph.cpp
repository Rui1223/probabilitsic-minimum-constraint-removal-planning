/*This cpp file defines a LabeledGraph class which we will use as the labeled graph in our problem 
formulation*/

#include <cstdio>
#include <cassert>
#include <cmath>
#include <iostream>
#include "LabeledGraph.hpp"


LabeledGraph_t::LabeledGraph_t(int row, int col)
{
	assert(row > 0);
	assert(col > 0);
	m_row = row;
	m_col = col;
	m_nNodes = m_row * m_col;
	load_graph();
	load_weights();
	graph_print();
	printf("-------------------------------\n");
}

void LabeledGraph_t::load_graph()
{
	printf("Build the graph now\n");
	specify_neighbors();
	specify_labels();
	printf("Finish building the graph\n");

}

void LabeledGraph_t::specify_neighbors()
{
	// we assume for each node, we have two neighbors to actively add by default
	// The right one and the bottom one
	for (int i=0; i < m_row; i++)
	{
		for (int j=0; j < m_col; j++)
		{
			m_nodeNeighbors.push_back(std::vector<int>());
		}
	}

	int iter = 0;
	for (int i=0; i < m_row; i++)
	{
		for (int j=0; j < m_col; j++)
		{
			if (i % m_row == m_row-1 and j % m_col == m_col-1)
			{
				// no neighbor to add since the node is at the right-bottom corner
			}
			else if (i % m_row == m_row-1 and j % m_col != m_col-1)
			{
				// the node is in the bottom line of the grid graph
				// only add the neighbor of its right
				m_nodeNeighbors[iter].push_back(iter+1);
				m_nodeNeighbors[iter+1].push_back(iter);
			}
			else if (i % m_row != m_row-1 and j % m_col == m_col-1)
			{
				// the node is in the rightmost line of the grid graph
				// only add the neighbor of its bottom
				m_nodeNeighbors[iter].push_back(iter+m_col);
				m_nodeNeighbors[iter+m_col].push_back(iter);
			}
			else
			{
				// add the neighbor of its right and bottom
				m_nodeNeighbors[iter].push_back(iter+1);
				m_nodeNeighbors[iter+1].push_back(iter);
				m_nodeNeighbors[iter].push_back(iter+m_col);
				m_nodeNeighbors[iter+m_col].push_back(iter);
			}
			iter++;
		}		
	}
}

void LabeledGraph_t::specify_labels()
{
	// we manually specify the labels for each edge
	// for those edges which don't have labels, we simply assign 0
	// for those noeds which don't even have an edgem, we can also assign 0
	/*
	for (int i=0; i <= pow(m_gridSize, 2)-1; i++)
	{
		m_edgeLabels.push_back(std::vector<std::vector<int>>());
		for (int j=0; j <= pow(m_gridSize, 2)-1; j++)
		{
			m_edgeLabels[i].push_back(std::vector<int>(1,0));
		}
	}
	*/

	for (int i=0; i <= m_nNodes-1; i++)
	{
		m_edgeLabels.push_back(std::vector<std::vector<int>>(m_nNodes,
			 std::vector<int>(1, 0)));
	}
	// manually assign the label
	m_edgeLabels[0][1].push_back(1);
	m_edgeLabels[1][0].push_back(1);
	m_edgeLabels[1][2].push_back(1);
	m_edgeLabels[1][7].push_back(1);
	m_edgeLabels[1][7].push_back(2);
	m_edgeLabels[2][1].push_back(1);
	m_edgeLabels[2][3].push_back(1);
	m_edgeLabels[2][8].push_back(2);
	m_edgeLabels[3][2].push_back(1);
	m_edgeLabels[3][4].push_back(1);
	m_edgeLabels[3][9].push_back(1);
	m_edgeLabels[3][9].push_back(3);
	m_edgeLabels[4][3].push_back(1);
	m_edgeLabels[4][5].push_back(1);
	m_edgeLabels[4][10].push_back(3);
	m_edgeLabels[5][4].push_back(1);
	m_edgeLabels[5][11].push_back(1);
	m_edgeLabels[6][7].push_back(2);
	m_edgeLabels[7][6].push_back(2);
	m_edgeLabels[7][1].push_back(1);
	m_edgeLabels[7][1].push_back(2);
	m_edgeLabels[7][8].push_back(2);
	m_edgeLabels[8][7].push_back(2);
	m_edgeLabels[8][2].push_back(2);
	m_edgeLabels[8][9].push_back(2);
	m_edgeLabels[8][9].push_back(3);
	m_edgeLabels[9][8].push_back(2);
	m_edgeLabels[9][8].push_back(3);
	m_edgeLabels[9][3].push_back(1);
	m_edgeLabels[9][3].push_back(3);
	m_edgeLabels[9][10].push_back(3);
	m_edgeLabels[10][9].push_back(3);
	m_edgeLabels[10][4].push_back(3);
	m_edgeLabels[10][11].push_back(3);
	m_edgeLabels[11][10].push_back(3);
	m_edgeLabels[11][5].push_back(1);
	/*
	m_edgeLabels[0][1].push_back(1);
	m_edgeLabels[0][3].push_back(1);
	m_edgeLabels[1][0].push_back(1);
	m_edgeLabels[3][0].push_back(1);
	m_edgeLabels[3][4].push_back(2);
	m_edgeLabels[3][6].push_back(1);
	m_edgeLabels[3][6].push_back(2);
	m_edgeLabels[4][3].push_back(2);
	m_edgeLabels[4][5].push_back(3);
	m_edgeLabels[4][7].push_back(2);
	m_edgeLabels[4][7].push_back(3);
	m_edgeLabels[5][4].push_back(3);
	m_edgeLabels[5][8].push_back(3);
	m_edgeLabels[6][3].push_back(1);
	m_edgeLabels[6][3].push_back(2);
	m_edgeLabels[6][7].push_back(2);
	m_edgeLabels[7][6].push_back(2);
	m_edgeLabels[7][4].push_back(2);
	m_edgeLabels[7][4].push_back(3);
	m_edgeLabels[7][8].push_back(2);
	m_edgeLabels[8][7].push_back(2);
	m_edgeLabels[8][5].push_back(3);
	*/
}

void LabeledGraph_t::load_weights()
{
	m_labelWeights.push_back(0.0);
	m_labelWeights.push_back(0.4);
	m_labelWeights.push_back(0.3);
	m_labelWeights.push_back(0.3);
}

void LabeledGraph_t::graph_print() 
{	
	for (int i=0; i <= m_nNodes-1; i++)
	{
		for (int j=0; j <= m_nNodes-1; j++)
		{
			for (auto const &element : m_edgeLabels[i][j])
			{
				std::cout << element << ",";
			}
			std::cout << "\t";
		}
		std::cout << std::endl;
	}

	printf("*********neighbors************\n");

	for (int k=0; k < m_nodeNeighbors.size(); k++)
	{
		for (auto const &e : m_nodeNeighbors[k])
		{
			std::cout << e << " ";
		}
		std::cout << "\n";
	}
	
}

LabeledGraph_t::~LabeledGraph_t() {}

