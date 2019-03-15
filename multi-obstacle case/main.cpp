#include <vector>
#include <iostream>
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <fstream>
#include <string> // std::string, std::to_string
#include <cstdlib> // std::rand, std::srand

bool check_subset(std::vector<int> set, std::vector<int> subset)
{
	// This function check whether a input set of labels is a subset of the m_currentLabels
	return ( std::includes(set.begin(), set.end(), subset.begin(), subset.end()) );	
}

int main()
{
	std::vector<int> a;
	std::vector<int> b{1};
	std::cout << check_subset(b, a) << std::endl;


	return 0;
}