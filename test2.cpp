/*test whether powerset works or not*/
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

// Declaring the type of Predicate that accept two pairs and return a bool
typedef std::function<bool(std::pair<std::vector<int>, double>, 
	std::pair<std::vector<int>, double>)> Comparator;

Comparator compFunctor = 
		[](std::pair<std::vector<int>, double> elem1, std::pair<std::vector<int>, double> elem2)
		{
			return elem1.second <= elem2.second; // compare weights, prefer less weight  
		};

std::vector<std::vector<int>> cal_powerSet(std::vector<int> set, int set_size)
{
	// This function takes a set and then compute the powerset of the set
	// which is a vector of sets (std::vector<std::vector<int>>)

	// first determines the size of the powerset that you will have
	// for each one's derivation we will do bitwise operation
	int powerSet_size = pow(2, set_size); // 2^n combinations
	std::vector<std::vector<int>> powerSet;
	for (int counter = 0; counter < powerSet_size; counter++)
	{
		std::vector<int> labels; // labels for a single combination
		for (int j=0; j < set_size; j++)
		{
			if ( counter & (1<<j) )
			{
				labels.push_back(set[j]);
			}
		}
		powerSet.push_back(labels);
	}
	return powerSet;

}

void print_label_combinations(std::vector<std::vector<int>> powerSet)
{
	std::cout << "print the label combinations\n";
	for (auto const &set : powerSet)
	{
		for (auto const &e : set)
		{
			std::cout << e <<",";
		}
		std::cout << "\n";
	}
}

std::vector<double> cal_weights(std::vector<std::vector<int>> powerSet, 
	std::vector<double> labelWeights)
{
	std::vector<double> weightCombinations;
	for (auto const &set : powerSet)
	{
		double weight = 0.0;
		for (auto const &label : set)
		{
			weight += labelWeights[label-1];
		}
		weightCombinations.push_back(weight);
	}
	return weightCombinations;
}

std::map<std::vector<int>, double> zip_combinations(std::vector<std::vector<int>>& a, 
	std::vector<double>& b)
{
	std::map<std::vector<int>, double> m;
	//assert(a.size() == b.size());
	for (int i=0; i < a.size(); ++i)
	{
		m[a[i]] = b[i];
	}
	return m;
}

int main()
{
	std::vector<double> labelWeights{0.5, 0.1, 0.2, 0.2};
	std::vector<int> labels = {1,2,3,4};
	std::vector<std::vector<int>> labelCombinations = cal_powerSet(labels, labels.size());
	std::vector<double> weightCombinations = cal_weights(labelCombinations, labelWeights);

	// zip label and weight combination
	std::map<std::vector<int>, double> map_combinations 
		= zip_combinations(labelCombinations, weightCombinations);

	std::set<std::pair<std::vector<int>, double>, Comparator> labelMap;

	
	// Declaring a set that will store the pairs using above comparison logic
	std::set<std::pair<std::vector<int>, double>, Comparator> labelMap(
		map_combinations.begin(), map_combinations.end(), compFunctor);

	//Iterate over the set you just come up with
	for (auto const &e : labelMap)
	{
		std::cout << "<";
		for (auto const &l : e.first)
		{
			std::cout << l << ",";
		}
		std::cout << "> :\t\t";
		std::cout << e.second << std::endl;
	}

	//print_label_combinations(labelCombinations);
	
	return 0;
}


