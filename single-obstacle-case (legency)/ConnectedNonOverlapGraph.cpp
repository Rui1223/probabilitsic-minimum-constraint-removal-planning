/*This cpp file defines a ConnectedGraph class which we will use as the labeled graph 
in our problem formulation*/

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

#include "ConnectedNonOverlapGraph.hpp"


Comparator compFunctor = 
		[](std::pair<std::vector<int>, double> elem1, std::pair<std::vector<int>, double> elem2)
		{
			return elem1.second <= elem2.second; // compare weights, prefer less weight  
		};

ConnectedNonOverlapGraph_t::ConnectedNonOverlapGraph_t(int row, int col, 
									int nlabels, double probPerLabel)
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
	m_nlabels = nlabels;
	m_probPerLabel = probPerLabel;

	std::cout << "probPerLabel: " << m_probPerLabel << std::endl;
	// based on the probability that a label is assigned to an edge, compute how many expansions 
	// per label are needed 
	m_nExpansion = m_probPerLabel * m_nEdges;  // keep the density to be 60%
	std::cout << "n_expansion: " << m_nExpansion << "\n";
	std::cout << "Expected density: " << m_probPerLabel * m_nlabels << "\n";
	m_nmarked = 0;
	// specify the number of labels and their corresponding weights
	// Based on weighted labels, build the labelMap which maps a set of labels to weights
	load_labels();
	load_weights();
	cal_labelMap();

	// construct the graph 
	load_graph();

	printf("--------------------------------------------------\n");
	//print_labelMap();
}

void ConnectedNonOverlapGraph_t::load_graph()
{
	//printf("Build the graph now\n");
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
		m_marked.push_back(std::vector<bool>(m_nNodes, false));
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

	label_graph();
	std::cout << "Acutal density: " << double(m_nmarked) / m_nEdges << "\n";
	//printf("Finish building the graph\n");

}

void ConnectedNonOverlapGraph_t::label_graph()
{
	// for each label
	for (auto const &l : m_labels)
	{
		// random pick up a node to expand (in a BFS search)
		int BF_start = random_generate_integer(0, m_nNodes-1);
		//std::cout << "start for label " + std::to_string(l) + ": " << BF_start << "\n";
		bool success = BFSearch(BF_start, l);
		while (!success)
		{
			int BF_start = random_generate_integer(0, m_nNodes-1);
			//std::cout << "start for label " + std::to_string(l) + ": " << BF_start << "\n";
			success = BFSearch(BF_start, l);
		}
	}
}

bool ConnectedNonOverlapGraph_t::BFSearch(int BF_start, int l)
{
	bool success = false;
	std::queue<int> q;
	std::vector<bool> expanded(m_nNodes, false);
	std::set<std::pair<int, int>> edgesToLabel;
	q.push(BF_start);

	int counter = 0;
	while(!q.empty())
	{
		int current = q.front(); // get the top of the queue
		q.pop();
		if (expanded[current]==true)
		{
			// no need to expand from that current node since it has been expanded before
			continue;
		}
		// get neighbors of the current node
		std::vector<int> neighbors = m_nodeNeighbors[current];
		for (auto const &neighbor : neighbors)
		{
			if (expanded[neighbor] or m_marked[current][neighbor] == true) { continue; }
			// Otherwise, put the edge between current and neighbor 
			// into edgesToLabel (wait to be labeled) and mark the edge to be labeled in m_marked
			edgesToLabel.insert(std::pair<int, int>(current, neighbor));
			m_marked[current][neighbor] = true;
			m_marked[neighbor][current] = true;
			q.push(neighbor);
			// count for each label process
			counter++;
			if (counter == m_nExpansion) 
			{
				success = true; 
				break; 
			}
		}
		if (counter == m_nExpansion) { break; }
		expanded[current]=true;
	}
	// You are reaching here either because
	// 1. you hit the counter limit, and it means you finish expansion OR
	// 2. the q is empty before the counter goes to m_nExpansion
	if (success)
	{
		// now label the edges
		for (auto const &pair : edgesToLabel)
		{
			m_nmarked++;
			int n1 = pair.first;
			int n2 = pair.second;
			m_edgeLabels[n1][n2].push_back(l);
			m_edgeLabels[n2][n1].push_back(l);
		}
	}
	else
	{
		// need to revert the information we change during this failure label process
		// specifically m_marked
		for (auto const &pair : edgesToLabel)
		{
			int n1 = pair.first;
			int n2 = pair.second;
			m_marked[n1][n2] = false;
			m_marked[n2][n1] = false;			
		}
	}
	return success;
}


void ConnectedNonOverlapGraph_t::write_graph(std::string file_dir)
{
	// This function write the constructed graph into a text file so as to be loaded by a python
	// script so as to visualize using matplotlib

	// to write in a text file (ostream)
	// we need to loop through the neighbor list, and then access to corresponding labels
	std::ofstream file_(file_dir);
	if (file_.is_open())
	{
		// Write in the 1st line the size and the density of the grid graph
		file_ << m_row << " " << m_col << " " << double(m_nmarked) / m_nEdges <<"\n";
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

void ConnectedNonOverlapGraph_t::load_labels()
{
	for (int ii=1; ii <= m_nlabels; ii++)
	{
		m_labels.push_back(ii);
	}
}

void ConnectedNonOverlapGraph_t::load_weights()
{
	std::random_device rd{};
	std::mt19937 gen{rd()};
	double r = 0.0;

	std::normal_distribution<> d{5.0,5.0};	

	for (int kk = 0; kk < m_nlabels; kk++)
	{
		double temp = d(gen);
		while (temp <= 0) { temp = d(gen); }
		m_labelWeights.push_back(temp);
		r += temp;
	}
	// normalize
	m_labelWeights /= r;
}

void ConnectedNonOverlapGraph_t::print_graph() 
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

double ConnectedNonOverlapGraph_t::compute_weight(std::vector<int> currentLabels)
{
	double currentWeights = 0.0;
	for (auto const &label : currentLabels)
	{
		currentWeights += m_labelWeights[label-1]; 
	}
	return currentWeights;

}

std::vector<double> ConnectedNonOverlapGraph_t::compute_weights()
{
	std::vector<double> weightCombinations;
	for (auto const &set : m_labelCombinations)
	{
		double weight = compute_weight(set);
		weightCombinations.push_back(weight);
	}

	return weightCombinations;
}

void ConnectedNonOverlapGraph_t::cal_powerSet()
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

std::map<std::vector<int>, double> 
	ConnectedNonOverlapGraph_t::zip_combinations(std::vector<double>& b)
{
	std::map<std::vector<int>, double> m;
	//assert(a.size() == b.size());
	for (int i=0; i < m_labelCombinations.size(); ++i)
	{
		m[m_labelCombinations[i]] = b[i];
	}
	return m;	
}

void ConnectedNonOverlapGraph_t::cal_labelMap()
{
	cal_powerSet();
	std::vector<double> weightCombinations = compute_weights();

	// zip label and weight combination
	std::map<std::vector<int>, double> map_combinations 
		= zip_combinations(weightCombinations);

	m_labelMap = std::set<std::pair<std::vector<int>, double>, Comparator>(
		map_combinations.begin(), map_combinations.end(), compFunctor);
}

void ConnectedNonOverlapGraph_t::print_labelMap()
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


ConnectedNonOverlapGraph_t::~ConnectedNonOverlapGraph_t() {}

int random_generate_integer(int min, int max)
{

	return rand() % (max + 1 - min) + min;
}

void operator/=(std::vector<double> &v, double d)
{
	for (auto &e : v)
	{
		e /= d;
	}
}