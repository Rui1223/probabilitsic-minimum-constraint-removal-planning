#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <string> // std::string, std::to_string
#include <vector>

int main()
{
	std::vector<int> a{1,2,3,4,5};
	std::vector<int> b;
	b = a * 2;

	for (auto const &e: b)
	{
		std::cout << e << " ";
	}
	std::cout << "\n";

	return 0;
}