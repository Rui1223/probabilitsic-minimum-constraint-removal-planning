/*This cpp file defines a ToyGraph class which we will use in a multi-obstacle scenario.
formulation*/

#include <cstdio>
#include <cassert>
#include <cmath>  // pow(), sqrt()
#include <iostream> 
#include <fstream> // stream class to write on files
#include <cstdio> // printf() 
#include <vector> // std::vector
#include <algorithm> // std::set_union, std::sort
#include <bitset> // for bitwise operation
#include <map> // std::map
#include <functional>
#include <set>
#include <cstdlib> // for std::srand()
#include <string> // std::string, std::to_string
#include <queue>
#include <random>
//#include <multiset>
#include "ToyGraph.hpp"


// Driver function to sort the vector elements 
// by second element of pairs 
bool sortbysec(const std::pair<std::vector<int>, double> &p1, 
				const std::pair<std::vector<int>, double> &p2)
{
	return (p1.second >= p2.second);
}

ToyGraph_t::ToyGraph_t(int row, int col, std::vector<int> nlabelsPerObs)
{
	printf("--------------------------------------------------\n");
	// specify the size of the graph
	assert(row > 0);
	assert(col > 0);
	m_row = row;
	m_col = col;
	m_nNodes = m_row * m_col;
	m_nEdges = (m_col-1)*m_row + (m_row-1)*m_col;
	
	// label information
	m_nlabelsPerObs = nlabelsPerObs;
	m_nobstacles = m_nlabelsPerObs.size();
	m_nTotallabels = 0;
	for (auto const &e : m_nlabelsPerObs)
	{
		m_nTotallabels += e;
	}
	
	// given #labels per obstacle, assign weight to each label
	// so that we end up knowing which obs a label belongs to and its corresponding weight 
	assign_weight_per_label();
	cal_labelMap();
	print_labelMap();

	// construct the graph 
	load_graph();
	//print_graph();

	printf("--------------------------------------------------\n");

}

void ToyGraph_t::load_graph()
{
	printf("Build the graph now\n");
	// This is the function to construct a random weighted labeled graph with 2 procedures
	// 1. specify edges (achieved by specifying neighbors)
	// 2. specify labels
	// we assume for each node, we have two neighbors to actively add by default
	// The right one and the bottom one

	int iter = 0;
	// first create empty neighbor and label vector for each node
	while (iter != m_nNodes)
	{
		m_nodeNeighbors.push_back(std::vector<int>());
		m_edgeLabels.push_back(std::vector<std::vector<int>>(m_nNodes,
			 std::vector<int>()));
		iter++;
	}

	int currentID = 0; // the id of the current node
	while (currentID != m_nNodes)
	{
		// add neighbors (in forms of their ids) of the current node and label the edge
		// between current node and its neighbors.

		// if the node is a rightmost and bottommost one
		if (currentID / m_col == m_row-1 and currentID % m_col == m_col-1)
		{
			// no neighbor to add since the node is at the right-bottom corner
		}
		// if the node is a rightmost one
		else if (currentID / m_col != m_row-1 and currentID % m_col == m_col-1)
		{
			// the node is in the rightmost line of the grid graph
			// only add the neighbor of its bottom	
			m_nodeNeighbors[currentID].push_back(currentID+m_col);
			m_nodeNeighbors[currentID+m_col].push_back(currentID);
		}
		// if the node is a bottommost one
		else if (currentID / m_col == m_row-1 and currentID % m_col != m_col-1)
		{
			// the node is in the bottom line of the grid graph
			// only add the neighbor of its right
			m_nodeNeighbors[currentID].push_back(currentID+1);
			m_nodeNeighbors[currentID+1].push_back(currentID);
					
		}
		// if the node is a normal one (should have two neighbors)
		else
		{
			// add the neighbor of its right and bottom
			m_nodeNeighbors[currentID].push_back(currentID+1);
			m_nodeNeighbors[currentID+1].push_back(currentID);
			m_nodeNeighbors[currentID].push_back(currentID+m_col);
			m_nodeNeighbors[currentID+m_col].push_back(currentID);
		}
		//std::cout << "working on next node!\n";
		currentID++; // start working on the next node
	}

	// 2. manually assign the label (toy problem for test purposes)
	m_edgeLabels[0][1].push_back(0);
	m_edgeLabels[1][0].push_back(0);
	m_edgeLabels[1][2].push_back(0);
	m_edgeLabels[1][7].push_back(0);
	m_edgeLabels[1][7].push_back(2);
	m_edgeLabels[2][1].push_back(0);
	m_edgeLabels[2][3].push_back(0);
	m_edgeLabels[2][8].push_back(2);
	m_edgeLabels[3][2].push_back(0);
	m_edgeLabels[3][4].push_back(0);
	m_edgeLabels[3][9].push_back(0);
	m_edgeLabels[3][9].push_back(1);
	m_edgeLabels[4][3].push_back(0);
	m_edgeLabels[4][5].push_back(0);
	m_edgeLabels[4][10].push_back(3);
	m_edgeLabels[5][4].push_back(0);
	m_edgeLabels[5][11].push_back(0);
	m_edgeLabels[5][11].push_back(3);
	m_edgeLabels[6][7].push_back(2);
	m_edgeLabels[7][6].push_back(2);
	m_edgeLabels[7][1].push_back(0);
	m_edgeLabels[7][1].push_back(2);
	m_edgeLabels[7][8].push_back(2);
	m_edgeLabels[8][7].push_back(2);
	m_edgeLabels[8][2].push_back(2);
	m_edgeLabels[8][9].push_back(1);
	m_edgeLabels[8][9].push_back(2);
	m_edgeLabels[9][8].push_back(1);
	m_edgeLabels[9][8].push_back(2);
	m_edgeLabels[9][3].push_back(0);
	m_edgeLabels[9][3].push_back(1);
	m_edgeLabels[9][10].push_back(1);
	m_edgeLabels[9][10].push_back(3);
	m_edgeLabels[10][9].push_back(1);
	m_edgeLabels[10][9].push_back(3);
	m_edgeLabels[10][4].push_back(3);
	m_edgeLabels[10][11].push_back(3);
	m_edgeLabels[11][10].push_back(3);
	m_edgeLabels[11][5].push_back(0);
	m_edgeLabels[11][5].push_back(3);

	printf("Finish building the graph\n");
}


// void ToyGraph_t::write_graph(std::string file_dir)
// {
// 	// This function write the constructed graph into a text file so as to be loaded by a python
// 	// script so as to visualize using matplotlib

// 	// to write in a text file (ostream)
// 	// we need to loop through the neighbor list, and then access to corresponding labels
// 	std::ofstream file_(file_dir);
// 	if (file_.is_open())
// 	{
// 		// Write in the 1st line the size and the density of the grid graph
// 		file_ << m_row << " " << m_col << " " << double(m_nmarked) / m_nEdges <<"\n";
// 		// Write in the 2nd line label information
// 		for (int tt=0; tt < m_nlabels; tt++)
// 		{
// 			file_ << m_labels[tt] << ":" << m_labelWeights[tt] << " ";
// 		}
// 		file_ << "\n";
// 		// start to write in every edge information (edge & labels)
// 		for (int currentNode = 0; currentNode < m_nNodes; currentNode++)
// 		{
// 			for (auto const &neighbor : m_nodeNeighbors[currentNode])
// 			{
// 				if (currentNode > neighbor)
// 				{
// 					continue;
// 				}
// 				// write in the currentNode ID and neighbor ID
// 				file_ << currentNode << " " << neighbor << " ";
// 				// then write in the labels
// 				if (!m_edgeLabels[currentNode][neighbor].empty())
// 				{
// 					int pp = 0;
// 					while (pp < m_edgeLabels[currentNode][neighbor].size()-1)
// 					{
// 						file_ << m_edgeLabels[currentNode][neighbor][pp] << ",";
// 						pp++;
// 					}
// 					file_ << m_edgeLabels[currentNode][neighbor][pp];
// 				}
// 				file_ << "\n";

// 			}
// 		}
// 		file_.close();
// 	} 
// }

void ToyGraph_t::assign_weight_per_label()
{
	// in toy graph, we manually assign weight
	m_labelWeights[0] = std::pair<int,double>(0, 0.4);
	m_labelWeights[1] = std::pair<int, double>(0, 0.6);
	m_labelWeights[2] = std::pair<int, double>(1, 0.1);
	m_labelWeights[3] = std::pair<int, double>(1, 0.9);
}


void ToyGraph_t::print_graph() 
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

double ToyGraph_t::compute_survival_currentLabels(std::vector<int> currentLabels)
{
	double currentSurvival = 1.0;
	std::vector<double> CollisionPerObs(m_nobstacles, 0.0);
	for (auto const &label : currentLabels)
	{
		CollisionPerObs[getLabelWeights(label).first] += getLabelWeights(label).second;
	}
	for (auto const &collision_prob : CollisionPerObs)
	{
		currentSurvival *= (1 - collision_prob); 
	}

	return currentSurvival;

}

std::vector<double> ToyGraph_t::compute_survival()
{
	std::vector<double> survivalCombinations;
	for (auto const &set : m_labelCombinations)
	{
		double survival = compute_survival_currentLabels(set);
		survivalCombinations.push_back(survival);
	}

	return survivalCombinations;
}

void ToyGraph_t::cal_powerSet()
{
	// This function takes a set and then compute the powerset of the set
	// which is a vector of sets (std::vector<std::vector<int>>)

	// first determines the size of the powerset that you will have
	// for each one's derivation we will do bitwise operation
	int powerSet_size = pow(2, m_nTotallabels); // 2^n combinations
	for (int counter = 0; counter < powerSet_size; counter++)
	{
		std::vector<int> labels; // labels for a single combination
		for (int j=0; j < m_nTotallabels; j++)
		{
			if ( counter & (1<<j) )
			{
				labels.push_back(j);
			}
		}
		m_labelCombinations.push_back(labels);
	}

}

void ToyGraph_t::cal_labelMap()
{
	cal_powerSet();
	std::vector<double> survivalCombinations = compute_survival();

	for (int kkk=0; kkk < survivalCombinations.size(); kkk++)
	{
		m_labelMap.push_back(std::pair<std::vector<int>, double>(m_labelCombinations[kkk], 
																	survivalCombinations[kkk]));
	}

	sort(m_labelMap.begin(), m_labelMap.end(), sortbysec);
}

void ToyGraph_t::print_labelMap()
{
	// std::cout << m_labelMap.size() << std::endl;
	// for (auto it = m_labelMap.begin(); it != m_labelMap.end();it++){
	// 	std::cout << "ttt" << std::endl;
	// }
	printf("*********labelMap************\n");
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


ToyGraph_t::~ToyGraph_t() {}


void operator/=(std::vector<double> &v, double d)
{
	for (auto &e : v)
	{
		e /= d;
	}
}

