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
	std::deque<int> aDeque;

	aDeque.push_back(1);
	aDeque.push_back(2);
	aDeque.push_back(3);

	std::cout << aDeque[1] << "\n";

	std::cout << aDeque.front() << "\n";

	aDeque.pop_front();

	std::cout << aDeque.front() << "\n";

	std::cout << aDeque[1] << "\n";

	return 0;
}

