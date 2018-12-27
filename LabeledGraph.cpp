/*This cpp file defines a LabeledGraph class which we will use as the labeled graph in our problem 
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
#include "LabeledGraph.hpp"
#include <cstdlib>

Comparator compFunctor = 
		[](std::pair<std::vector<int>, double> elem1, std::pair<std::vector<int>, double> elem2)
		{
			return elem1.second <= elem2.second; // compare weights, prefer less weight  
		};

LabeledGraph_t::LabeledGraph_t(int row, int col, int n_labels)
{
	// specify the size of the graph
	assert(row > 0);
	assert(col > 0);
	m_row = row;
	m_col = col;
	m_nNodes = m_row * m_col;
	
	m_nlabels = n_labels;
	m_percentLabelEdge = 0.4;
	// The probability of a label to be assigned to an edge is determined by the number of labels
	// and the percent of labeled edge we expect in the graph
	m_prob = 1 - exp(1.0/m_nlabels*log(1-m_percentLabelEdge));
	std::cout << "m_prob: " << m_prob << std::endl;

	// specify the number of labels and their corresponding weights
	// Based on weighted labels, build the labelMap which maps a set of labels to weights
	load_labels();
	load_weights();
	cal_labelMap();

	// construct the graph 
	load_graph();

	// print the basic information of the graph
	print_labelMap();
	//print_graph();
	printf("-------------------------------\n");
}

void LabeledGraph_t::load_graph()
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
	double r;
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
			// assign labels
			for (auto const &l : m_labels)
			{
				r = ((double) rand() / (RAND_MAX));
				if (r < m_prob)
				{
					// assign the label to that edge
					m_edgeLabels[currentID][currentID+m_col].push_back(l);
					m_edgeLabels[currentID+m_col][currentID].push_back(l); // should be identical
				}
			}		
		}
		// if the node is a bottommost one
		else if (currentID / m_col == m_row-1 and currentID % m_col != m_col-1)
		{
			// the node is in the bottom line of the grid graph
			// only add the neighbor of its right
			m_nodeNeighbors[currentID].push_back(currentID+1);
			m_nodeNeighbors[currentID+1].push_back(currentID);
			for (auto const &l : m_labels)
			{
				r = ((double) rand() / (RAND_MAX));
				if (r < m_prob)
				{
					// assign the label to that edge
					m_edgeLabels[currentID][currentID+1].push_back(l);
					m_edgeLabels[currentID+1][currentID].push_back(l); // should be identical
				}
			}		
		}
		// if the node is a normal one (should have two neighbors)
		else
		{
			// add the neighbor of its right and bottom
			m_nodeNeighbors[currentID].push_back(currentID+1);
			m_nodeNeighbors[currentID+1].push_back(currentID);
			for (auto const &l : m_labels)
			{
				r = ((double) rand() / (RAND_MAX));
				if (r < m_prob)
				{
					// assign the label to that edge
					m_edgeLabels[currentID][currentID+1].push_back(l);
					m_edgeLabels[currentID+1][currentID].push_back(l); // should be identical
				}
			}
			m_nodeNeighbors[currentID].push_back(currentID+m_col);
			m_nodeNeighbors[currentID+m_col].push_back(currentID);
			for (auto const &l : m_labels)
			{
				r = ((double) rand() / (RAND_MAX));
				if (r < m_prob)
				{
					// assign the label to that edge
					m_edgeLabels[currentID][currentID+m_col].push_back(l);
					m_edgeLabels[currentID+m_col][currentID].push_back(l); // should be identical
				}
			}
		}
		//std::cout << "working on next node!\n";
		currentID++; // start working on the next node
	}
	printf("Finish building the graph\n");

}

// void LabeledGraph_t::specify_labels()
// {
// 	// 2. manually assign the label (toy problem for test purposes)
// 	m_edgeLabels[0][1].push_back(1);
// 	m_edgeLabels[1][0].push_back(1);
// 	m_edgeLabels[1][2].push_back(1);
// 	m_edgeLabels[1][7].push_back(1);
// 	m_edgeLabels[1][7].push_back(2);
// 	m_edgeLabels[2][1].push_back(1);
// 	m_edgeLabels[2][3].push_back(1);
// 	m_edgeLabels[2][8].push_back(2);
// 	m_edgeLabels[3][2].push_back(1);
// 	m_edgeLabels[3][4].push_back(1);
// 	m_edgeLabels[3][9].push_back(1);
// 	m_edgeLabels[3][9].push_back(3);
// 	m_edgeLabels[4][3].push_back(1);
// 	m_edgeLabels[4][5].push_back(1);
// 	m_edgeLabels[4][10].push_back(3);
// 	m_edgeLabels[5][4].push_back(1);
// 	m_edgeLabels[5][11].push_back(1);
// 	m_edgeLabels[6][7].push_back(2);
// 	m_edgeLabels[7][6].push_back(2);
// 	m_edgeLabels[7][1].push_back(1);
// 	m_edgeLabels[7][1].push_back(2);
// 	m_edgeLabels[7][8].push_back(2);
// 	m_edgeLabels[8][7].push_back(2);
// 	m_edgeLabels[8][2].push_back(2);
// 	m_edgeLabels[8][9].push_back(2);
// 	m_edgeLabels[8][9].push_back(3);
// 	m_edgeLabels[9][8].push_back(2);
// 	m_edgeLabels[9][8].push_back(3);
// 	m_edgeLabels[9][3].push_back(1);
// 	m_edgeLabels[9][3].push_back(3);
// 	m_edgeLabels[9][10].push_back(3);
// 	m_edgeLabels[10][9].push_back(3);
// 	m_edgeLabels[10][4].push_back(3);
// 	m_edgeLabels[10][11].push_back(3);
// 	m_edgeLabels[11][10].push_back(3);
// 	m_edgeLabels[11][5].push_back(1);
// }


void LabeledGraph_t::write_graph()
{
	// This function write the constructed graph into a text file so as to be loaded by a python
	// script so as to visualize using matplotlib

	// to write in a text file (ostream)
	// we need to loop through the neighbor list, and then access to corresponding labels
	std::ofstream file_("graph.txt");
	if (file_.is_open())
	{
		// Write in the 1st line the size of the grid graph
		file_ << m_row << " " << m_col << "\n";
		// Write in the 2nd line label information
		for (int tt=0; tt < m_nlabels; tt++)
		{
			file_ << m_labels[tt] << ":" << m_labelWeights[tt] << " ";
		}
		file_ << "\n";
		// start to write in every edge information (edge & labels)
		for (int currentNode = 0; currentNode < m_nNodes; currentNode++)
		{
			for (auto const &neighbor : m_nodeNeighbors[currentNode])
			{
				if (currentNode > neighbor)
				{
					continue;
				}
				// write in the currentNode ID and neighbor ID
				file_ << currentNode << " " << neighbor << " ";
				// then write in the labels
				if (!m_edgeLabels[currentNode][neighbor].empty())
				{
					int pp = 0;
					while (pp < m_edgeLabels[currentNode][neighbor].size()-1)
					{
						file_ << m_edgeLabels[currentNode][neighbor][pp] << ",";
						pp++;
					}
					file_ << m_edgeLabels[currentNode][neighbor][pp];
				}
				file_ << "\n";

			}
		}
		file_.close();
	} 
}

void LabeledGraph_t::load_labels()
{
	for (int ii=1; ii <= m_nlabels; ii++)
	{
		m_labels.push_back(ii);
	}

	/*
	// manually set the labels
	m_labels.push_back(1);
	m_labels.push_back(2);
	m_labels.push_back(3);
	*/
}

void LabeledGraph_t::load_weights()
{

	double r = 0.0;
	int temp;

	for (int kk = 0; kk < m_nlabels; kk++)
	{
		temp = random_generate_integer(1, 5);
		m_labelWeights.push_back(temp);
		r += double(temp);
	}
	
	// now randomize them so that the sum of weights are equal to 1 (can be smaller than one)
	m_labelWeights /= r;


	/*
	// manually assign label weights
	m_labelWeights.push_back(0.4);
	m_labelWeights.push_back(0.3);
	m_labelWeights.push_back(0.3);
	*/
}

void LabeledGraph_t::print_graph() 
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

std::vector<double> LabeledGraph_t::compute_weights()
{
	std::vector<double> weightCombinations;
	for (auto const &set : m_labelCombinations)
	{
		double weight = compute_weight(set);
		weightCombinations.push_back(weight);
	}

	return weightCombinations;
}

void LabeledGraph_t::cal_powerSet()
{
	// This function takes a set and then compute the powerset of the set
	// which is a vector of sets (std::vector<std::vector<int>>)

	// first determines the size of the powerset that you will have
	// for each one's derivation we will do bitwise operation
	int powerSet_size = pow(2, m_labels.size()); // 2^n combinations
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
		m_labelCombinations.push_back(labels);
	}
}

std::map<std::vector<int>, double> LabeledGraph_t::zip_combinations(std::vector<double>& b)
{
	std::map<std::vector<int>, double> m;
	//assert(a.size() == b.size());
	for (int i=0; i < m_labelCombinations.size(); ++i)
	{
		m[m_labelCombinations[i]] = b[i];
	}
	return m;	
}

void LabeledGraph_t::cal_labelMap()
{
	cal_powerSet();
	std::vector<double> weightCombinations = compute_weights();

	// zip label and weight combination
	std::map<std::vector<int>, double> map_combinations 
		= zip_combinations(weightCombinations);

	m_labelMap = std::set<std::pair<std::vector<int>, double>, Comparator>(
		map_combinations.begin(), map_combinations.end(), compFunctor);
}

void LabeledGraph_t::print_labelMap()
{
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


LabeledGraph_t::~LabeledGraph_t() {}

int random_generate_integer(int min, int max)
{

	return std::rand() % (max + 1 - min) + min;
}

void operator/=(std::vector<double> &v, double d)
{
	for (auto &e : v)
	{
		e /= d;
	}
}

