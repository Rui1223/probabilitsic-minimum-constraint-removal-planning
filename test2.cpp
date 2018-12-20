/*test how we can extract all first item of a pair of a set*/

#include <set>
#include <vector>
#include <iostream>
#include <map>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <random>

std::ostream& operator<<(std::ostream &out, std::vector<double> v)
{
	out << "<";
	for (auto const &e : v)
	{
		out << e << "\t";
	}
	out << ">\n";
	return out;
}

int random_generate_integer(int min, int max)
{

	return std::rand() % (max + 1 - min) + min;
}

void operator/=(std::vector<double> &v, double d)
{
	for (auto &e : v)
	{
		e /= d;
	}
}


int main()
{
	std::vector<int> v{1,2,3,4};
	for (int i=0; i < 5; i++)
	{
		add_member();
	}
}

