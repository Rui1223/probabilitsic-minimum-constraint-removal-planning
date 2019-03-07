// test how to generate several number with large variance

// normal distribution
#include <iostream> 
//#include <cmath>
#include <algorithm> // std::sort
#include <cstring>
#include <random>
#include <vector>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <map>
#include <cassert>
#include <functional>
#include <set>
#include <queue>
#include <deque>

#include "Timer.hpp"

int random_generate_integer(int min, int max)
{

	return rand() % (max + 1 - min) + min;
}


int main()
{
	std::vector<int> v1{8,4,6,7,1};
	std::vector<int>::iterator it;

	for (auto const &v : v1)
	{
		std::cout << v << " ";
	}
	std::cout << "\n";

	return 0;
}

