#include <iostream> // std::cout 
#include <vector> // std::vector
#include <algorithm> // std::set_union, std::sort

void cout_vector(std::vector<int> s);

int main()
{
	std::vector<int> s1{15,10,5,20,25};
	std::vector<int> s2{50,40,30,20,10};

	// sort the sets first before applying union operation
	std::sort(s1.begin(), s1.end());
	std::sort(s2.begin(), s2.end());

	// Declaring resultant vector for union
	std::vector<int> v(s1.size()+s2.size());

	// using function set_union() to compute union of 2
	// containers v1 and v2 and store result in v
	auto it = std::set_union(s1.begin(), s1.end(), s2.begin(), s2.end(), v.begin());

	// resizing new container
	v.resize(it - v.begin());

	cout_vector(v);

	return 0;

}

void cout_vector(std::vector<int> s)
{
	for (auto const &e : s)
	{
		std::cout << e << " ";
	}
	std::cout << "\n";
}


