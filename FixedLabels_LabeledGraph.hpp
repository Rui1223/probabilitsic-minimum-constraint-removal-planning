/*The hpp files declares a new LabeledGraph, which is an extension of the LabeledGraph class*/
/*Basically so far it has a key new attribute: label map, which is a set where each item
  is a pair (labels, weights) and is sorted in order of ascending weights*/

#ifndef FIXEDLABELS_LABELEDGRAPH_H
#define FIXEDLABELS_LABELEDGRAPH_H

#include <iostream> // std::cout
#include <cstdio> // printf() 
#include <vector> // std::vector
#include <cmath> // pow()
#include <algorithm> // std::set_union, std::sort
#include <bitset> // for bitwise operation
#include <map>
#include <cassert>
#include <functional>
#include <set>

//#include "LabeledGraph.hpp"


// // Declaring the type of Predicate that accept two pairs and return a bool
// typedef std::function<bool(std::pair<std::vector<int>, double>, 
// 	std::pair<std::vector<int>, double>)> Comparator;

// Comparator compFunctor = 
// 		[](std::pair<std::vector<int>, double> elem1, std::pair<std::vector<int>, double> elem2)
// 		{
// 			return elem1.second <= elem2.second; // compare weights, prefer less weight  
// 		};



class FixedLabels_LabeledGraph_t : public LabeledGraph_t
{
	//std::set<std::pair<std::vector<int>, double>, Comparator> labelMap;

public:
	FixedLabels_LabeledGraph_t() {};
	FixedLabels_LabeledGraph_t(int row, int col);

};

#endif