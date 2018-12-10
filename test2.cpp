#include <iostream> // std::cout
#include <cstdio> // printf() 
#include <vector> // std::vector
#include <cmath> // pow()
#include <algorithm> // std::set_union, std::sort
#include <bitset> // for bitwise operation


std::vector<std::vector<int>> calPowerSet(std::vector<int> set, int set_size)
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

int main()
{
	std::vector<int> labels = {1,2,3,4};
	std::vector<std::vector<int>> labelCombinations = calPowerSet(labels, labels.size());
	print_label_combinations(labelCombinations);
	return 0;
}

