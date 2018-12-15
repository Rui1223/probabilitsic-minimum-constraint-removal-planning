/*test whether subset check works or not*/

#include <algorithm>
#include <iostream>

// bool check_subset(std::vector<int> &l1, std::vector<int> &l2)
// {
// 	return ( std::includes(l1.begin(), l1.end(),
// 							l2.begin(), l2.end()) );
// }

std::ostream& operator<<(std::ostream &out, const std::vector<int> &v)
{
	for (auto const &item : v)
	{
		out << item << " ";
	}
	return out;
}

int main()
{
	std::vector<int> l1{1,2,3};
	std::vector<int> l2{};
	// bool isSubset = check_subset(l1, l2);
	std::cout << "<" << l2 << ">\n";
	return 0;
}



