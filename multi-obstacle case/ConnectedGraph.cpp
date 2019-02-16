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

#include "ConnectedGraph.hpp"
#include "Timer.hpp"


// Driver function to sort the vector elements 
// by second element of pairs 
bool sortbysec_connectedGraph(const std::pair<std::vector<int>, double> &p1, 
				const std::pair<std::vector<int>, double> &p2)
{
	return (p1.second < p2.second);
}

ConnectedGraph_t::ConnectedGraph_t(int row, int col, std::vector<int> nlabelsPerObs, 
																			double probPerLabel)
{
	Timer tt;
	tt.reset();
	printf("--------------------------------------------------\n");
	// specify the size of the graph
	assert(row > 0);
	assert(col > 0);
	m_row = row;
	m_col = col;
	m_nNodes = m_row * m_col;
	m_nEdges = (m_col-1)*m_row + (m_row-1)*m_col;
	std::cout << "m_nEdges: " << m_nEdges << std::endl;
	
	// label information
	m_nlabelsPerObs = nlabelsPerObs;
	m_nobstacles = m_nlabelsPerObs.size();
	m_nTotallabels = 0;
	for (auto const &e : m_nlabelsPerObs)
	{
		m_nTotallabels += e;
	}	
	m_probPerLabel = probPerLabel;

	std::cout << "probPerLabel: " << m_probPerLabel << std::endl;
	// based on the probability that a label is assigned to an edge, compute how many expansions 
	// per label are needed 
	// m_nExpansion = round(m_nEdges * (1 - pow(1 - m_probPerLabel, m_nlabels))) / m_nlabels;
	m_nExpansion = round(m_nEdges * probPerLabel *1.0 / m_nTotallabels);
	std::cout << "n_expansion: " << m_nExpansion << "\n";
	std::cout << "Expected density: " << probPerLabel << "\n";
	m_nmarked = 0;

	// given #labels per obstacle, assign weight to each label
	// so that we end up knowing which obs a label belongs to and its corresponding weight 
	assign_weight_per_label();
	cal_labelMap();
	//print_labelMap();

	// construct the graph 
	load_graph();
	//print_graph();

	std::cout << "Time to build and label the graph: " << tt.elapsed() << " seconds\n";

	printf("--------------------------------------------------\n");
}

void ConnectedGraph_t::load_graph()
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

std::pair<int, int> ConnectedGraph_t::getLoc(int node_idx)
{
	return std::pair<int, int>(node_idx / m_col, node_idx % m_col);
}

bool ConnectedGraph_t::is_close(std::pair<int, int> &a, 
					std::vector<std::pair<double, double>> &b, int obs, double threshold)
{
	bool isClose = false;
	for (int pp=0; pp < obs; pp++)
	{
		if ( dist(a, b[pp]) < threshold)
		{
			isClose = true;
			break;
		}
	} 
	return isClose;
}

double ConnectedGraph_t::dist(std::pair<int, int> &a, std::pair<int, int> b)
{
	return sqrt(pow(a.first-b.first, 2) + pow(a.second-b.second, 2));
}


void ConnectedGraph_t::label_graph()
{
	// for each label
	//for (auto const &l : m_nTotallabels)

	// for (int ll=0; ll < m_nTotallabels; ll++)
	// {
	// 	// random pick up a node to expand (in a BFS search)
	// 	int BF_start = random_generate_integer(0, m_nNodes-1);
	// 	//std::cout << "start per label: " << BF_start << "\n"; 
	// 	BFSearch(BF_start, ll);
	// }
	int current_label_idx = 0;
	std::vector<std::pair<int, int>> mean_obs(m_nobstacles, std::pair<int, int>(0, 0));
	bool firstTimeObs;
	int BF_start;
	std::pair<int, int> BF_start_loc;
	double dist_threshold1 = m_nExpansion / 10;
	double dist_threshold2 = m_nExpansion / 6;
	for (int obs=0; obs < m_nobstacles; obs++)
	{
		// we are dealing with a new obstacle
		int temp_nlabels = m_nlabelsPerObs[obs];
		firstTimeObs = true;
		std::pair<int, int> temp_BF_loc{0, 0};
		//dist_threshold2 -= m_nExpansion / 30;

		for (int ii=0; ii<temp_nlabels; ii++)
		{
			if (current_label_idx==0) // the 1st label of the first obstacle
			{
				BF_start = random_generate_integer(0, m_nNodes-1);
				BF_start_loc = getLoc(BF_start);
				firstTimeObs = false;
				temp_BF_loc = BF_start_loc;
			}
			else if (firstTimeObs) // first time entering an obstacle (except the 1st obstacle)
			{
				// we want the obs not close to previous ones
				do
				{
					BF_start = random_generate_integer(0, m_nNodes-1);
					BF_start_loc = getLoc(BF_start);
				}
				while ( is_close(BF_start_loc, mean_obs, obs, dist_threshold2) );

				firstTimeObs = false;
				temp_BF_loc = BF_start_loc;
			}
			else // working among label (not the first one of any obstacles)
			{
				// we want labels of the same obstacle to be close to each other
				do
				{
					BF_start = random_generate_integer(0, m_nNodes-1);
					BF_start_loc = getLoc(BF_start);
				}
				while ( dist(BF_start_loc, temp_BF_loc) > dist_threshold1 );
			}

			// please print the BF_start
			std::cout << current_label_idx << ": " << "(" << BF_start_loc.first << "," 
																<< BF_start_loc.second << ")\n";

			mean_obs[obs].first += BF_start_loc.first;
			mean_obs[obs].second += BF_start_loc.second;
			// Now it's the time for calling BFSearch() with our well tested BF_start
			BFSearch(BF_start, current_label_idx);
			current_label_idx++;
		}
		mean_obs[obs].first /= temp_nlabels;
		mean_obs[obs].second /= temp_nlabels;
	}


}

void ConnectedGraph_t::BFSearch(int BF_start, int l)
{
	std::queue<int> q;
	std::vector<bool> expanded(m_nNodes, false);
	q.push(BF_start);

	int counter = 0;
	while(true)
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
			if (expanded[neighbor]) { continue; }
			// Otherwise, label the edge between current and neighbor
			// and push the neighbor to the open list
			m_edgeLabels[current][neighbor].push_back(l);
			m_edgeLabels[neighbor][current].push_back(l);
			if (m_marked[current][neighbor] == false) 
			{
				m_marked[current][neighbor] = true;
				m_marked[neighbor][current] = true;
				m_nmarked++;
			}
			q.push(neighbor);
			// count for each label process
			counter++;
			if (counter == m_nExpansion) { break; }
		}
		if (counter == m_nExpansion) { break; }
		expanded[current]=true;
	}
}


void ConnectedGraph_t::write_graph(std::string file_dir)
{
	// This functions write the constructed graph into a text file so as to be loaded by a python
	// script so as to visualize using matplotlib

	// to write in a text file (ostream)
	// we need to loop through the neighbor list, and then access to corresponding labels
	std::ofstream file_(file_dir);
	if (file_.is_open())
	{
		// Write in the 1st line the size and the density of the grid graph
		file_ << m_row << " " << m_col << " " << double(m_nmarked) / m_nEdges <<"\n";
		// Write in the 2nd line label information
		for (int tt=0; tt < m_nTotallabels; tt++)
		{
			file_ << tt << ":" << m_labelWeights[tt].second << " ";
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


std::vector<double> ConnectedGraph_t::load_weights(int nlabels)
{
	std::random_device rd{};
	std::mt19937 gen{rd()};
	double r = 0.0;

	std::normal_distribution<> d{5.0,5.0};

	std::vector<double> temp_weights; 

	for (int kk = 0; kk < nlabels; kk++)
	{
		double temp = d(gen);
		while (temp <= 0) { temp = d(gen); }
		temp_weights.push_back(temp);
		r += temp;
	}
	// normalize
	temp_weights /= r;
	return temp_weights;
}

void ConnectedGraph_t::assign_weight_per_label()
{
	// fill in the m_labelWeights
	int current_label_idx = 0;
	for (int obs=0; obs < m_nobstacles; obs++)
	{
		///////// for each obstacle /////////
		// first get #labels the current obstacle has
		int nlabels = m_nlabelsPerObs[obs];
		std::vector<double> weightsObs = load_weights(nlabels);
		for (auto const w: weightsObs)
		{
			m_labelWeights[current_label_idx] = std::pair<int, double>(obs, w);
			current_label_idx++;
		}

	} 
}

void ConnectedGraph_t::print_graph() 
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

double ConnectedGraph_t::compute_survival_currentLabels(std::vector<int> currentLabels)
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

std::vector<double> ConnectedGraph_t::compute_survival()
{
	std::vector<double> survivalCombinations;
	for (auto const &set : m_labelCombinations)
	{
		double survival = compute_survival_currentLabels(set);
		survivalCombinations.push_back(survival);
	}

	return survivalCombinations;
}

void ConnectedGraph_t::cal_powerSet()
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

void ConnectedGraph_t::cal_labelMap()
{
	cal_powerSet();
	std::vector<double> survivalCombinations = compute_survival();

	for (int kkk=0; kkk < survivalCombinations.size(); kkk++)
	{
		m_labelMap.push_back(std::pair<std::vector<int>, double>(m_labelCombinations[kkk], 
																	survivalCombinations[kkk]));
	}

	sort(m_labelMap.begin(), m_labelMap.end(), sortbysec_connectedGraph);
}

void ConnectedGraph_t::print_labelMap()
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
	printf("***************************\n");
}


ConnectedGraph_t::~ConnectedGraph_t() {}

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