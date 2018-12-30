#include <set>
#include <iostream>


int main()
{
	std::set<std::pair<int, int>> set;
	set.insert(std::pair<int, int>(10,15));
	set.insert(std::pair<int, int>(20,17));

	for (auto const &pair : set)
	{
		std::cout << pair.first << "\t" << pair.second << "\n";
	}


	return 0;
}