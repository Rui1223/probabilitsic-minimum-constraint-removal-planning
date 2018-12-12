/*This cpp file defines a LabeledGraph class which we will use as the labeled graph in our problem 
formulation*/

#include <cstdio>
#include <cassert>
#include <cmath>  // pow()
#include <iostream>
#include <cstdio> // printf() 
#include <vector> // std::vector
#include <algorithm> // std::set_union, std::sort
#include <bitset> // for bitwise operation
#include <map> // std::map
#include <functional>
#include <set>
#include "LabeledGraph.hpp"

Comparator compFunctor = 
		[](std::pair<std::vector<int>, double> elem1, std::pair<std::vector<int>, double> elem2)
		{
			return elem1.second <= elem2.second; // compare weights, prefer less weight  
		};

LabeledGraph_t::LabeledGraph_t(int row, int col)
{
	assert(row > 0);
	assert(col > 0);
	m_row = row;
	m_col = col;
	m_nNodes = m_row * m_col;
	load_graph();
	load_labels();
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
	// for those edges which don't have labels, we simply assign empty vector
	// for those noeds which don't even have an edgem, we can also assign empty vector


	for (int i=0; i <= m_nNodes-1; i++)
	{
		m_edgeLabels.push_back(std::vector<std::vector<int>>(m_nNodes,
			 std::vector<int>()));
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

void LabeledGraph_t::load_labels()
{
	m_labels.push_back(1);
	m_labels.push_back(2);
	m_labels.push_back(3);
}

void LabeledGraph_t::load_weights()
{
	m_labelWeights.push_back(0.4);
	m_labelWeights.push_back(0.3);
	m_labelWeights.push_back(0.3);
}

void LabeledGraph_t::graph_print() 
{
	printf("*********edgeLabels***********\n");

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

double LabeledGraph_t::compute_weight(std::vector<int> currentLabels)
{
	double currentWeights = 0.0;
	for (auto const &label : currentLabels)
	{
		currentWeights += m_labelWeights[label-1]; 
	}
	return currentWeights;

}

std::vector<double> LabeledGraph_t::compute_weights(std::vector<std::vector<int>> powerSet)
{
	std::vector<double> weightCombinations;
	for (auto const &set : powerSet)
	{
		double weight = compute_weight(set);
		weightCombinations.push_back(weight);
	}

	return weightCombinations;
}

std::vector<std::vector<int>> LabeledGraph_t::cal_powerSet()
{
	// This function takes a set and then compute the powerset of the set
	// which is a vector of sets (std::vector<std::vector<int>>)

	// first determines the size of the powerset that you will have
	// for each one's derivation we will do bitwise operation
	int powerSet_size = pow(2, m_labels.size()); // 2^n combinations
	std::vector<std::vector<int>> powerSet;
	for (int counter = 0; counter < powerSet_size; counter++)
	{
		std::vector<int> labels; // labels for a single combination
		for (int j=0; j < m_labels.size(); j++)
		{
			if ( counter & (1<<j) )
			{
				labels.push_back(m_labels[j]);
			}
		}
		powerSet.push_back(labels);
	}
	return powerSet;
}

std::map<std::vector<int>, double> LabeledGraph_t::zip_combinations(
	std::vector<std::vector<int>>& a, std::vector<double>& b)
{
	std::map<std::vector<int>, double> m;
	//assert(a.size() == b.size());
	for (int i=0; i < a.size(); ++i)
	{
		m[a[i]] = b[i];
	}
	return m;	
}

void LabeledGraph_t::cal_labelMap()
{
	std::vector<std::vector<int>> labelCombinations = cal_powerSet();
	std::vector<double> weightCombinations = compute_weights(labelCombinations);

	// zip label and weight combination
	std::map<std::vector<int>, double> map_combinations 
		= zip_combinations(labelCombinations, weightCombinations);

	m_labelMap = std::set<std::pair<std::vector<int>, double>, Comparator>(
		map_combinations.begin(), map_combinations.end(), compFunctor);
}

void LabeledGraph_t::print_labelMap()
{
	//Iterate over the set you just come up with
	for (auto const &e : m_labelMap)
	{
		std::cout << "<";
		for (auto const &l : e.first)
		{
			std::cout << l << ",";
		}
		std::cout << "> :\t\t";
		std::cout << e.second << std::endl;
	}	
}


LabeledGraph_t::~LabeledGraph_t() {}

