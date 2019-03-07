#include <iostream> 
#include <fstream>
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
	// int a = 3;
	// int b = 5;
	// int c = 4;
	// int d = 6;
	// std::string file_dir = "./justatest.txt";
	// std::ofstream file_(file_dir);
	// if (file_.is_open())
	// {
	// 	file_ << a << " " << b << "\n";
	// }

	// if(file_.is_open())
	// {
	// 	file_ << c << " " << d << "\n";
	// }

	int a = 5;
	int b;
	if (a==5)
	{
		b = a+1;

	}

	std::cout << b << "\n";


	return 0;
	// std::vector<int> v1{50,90,40,90,80,60,70};
	// std::vector<int>::iterator it;

	// int a = 40;

	// if ( std::find(v1.begin(), v1.end(), a) != v1.end() )
	// {
	// 	std::cout << std::distance(v1.begin(), std::find(v1.begin(), v1.end(), a)) << "\n";
	// }

	// for (auto const &v : v1)
	// {
	// 	std::cout << v << " ";
	// }
	// std::cout << "\n";

	// int deletion_counter = 0;

	// for (int gg=0; gg < v1.size(); gg++)
	// {
	// 	if (gg==1 or gg==2 or gg==4)
	// 	{
	// 		v1.erase(v1.begin() + gg - deletion_counter);
	// 		deletion_counter++;
	// 	}
	// }

	// std::cout << "-------------------\n";

	// for (auto const &v : v1)
	// {
	// 	std::cout << v << " ";
	// }
	// std::cout << "\n";	
}

