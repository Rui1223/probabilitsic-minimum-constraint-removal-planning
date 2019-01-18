// test how to generate several number with large variance

// normal distribution
#include <iostream> 
#include <algorithm> // std::sort
#include <cstring>
#include <random>
#include <vector>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <map>
//#include <cassert>
#include <functional>
#include <set>

#include "Timer.hpp"

// Declaring the type of Predicate that accept two pairs and return a bool
typedef std::function<bool(std::pair<std::vector<int>, double>, 
	std::pair<std::vector<int>, double>)> Comparator;

Comparator compFunctor = 
		[](std::pair<std::vector<int>, double> elem1, std::pair<std::vector<int>, double> elem2)
		{
			return elem1.second <= elem2.second; // compare weights, prefer less weight  
		};

void operator/=(std::vector<double> &v, double d)
{
	for (auto &e : v)
	{
		e /= d;
	}
}

std::ostream& operator<<(std::ostream &out, const std::vector<int> &v)
{
	out << "<"; 
	for (auto const &item : v)
	{
		out << item << " ";
	}
	out << ">";
	return out;
}


std::vector<int> commonSet(std::vector<int> v1, std::vector<int> v2)
{
	std::vector<int> s;
	std::sort(v1.begin(), v1.end());
	std::sort(v2.begin(), v2.end());
	std::set_intersection(v1.begin(),v1.end(),v2.begin(),v2.end(), back_inserter(s));
	return s;
}

std::vector<int> label_intersection(std::vector<int> s1, std::vector<int> s2, 
										std::vector<int> s3, std::vector<int> s4)
{
	std::vector<std::vector<int>> v;
	v.push_back(s1);
	v.push_back(s2);
	v.push_back(s3);
	v.push_back(s4);
	std::vector<int> s = s1;
	for (int kk=1; kk < v.size(); kk++)
	{
		s = commonSet(s, v[kk]);

	}
	return s;
}

std::vector<int> label_union(std::vector<int> s1, std::vector<int> s2)
{
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
	return v;	
}

bool check_subset(std::vector<int> set, std::vector<int> subset)
{
	// This function check whether a input set of labels is a subset of the m_currentLabels
	return ( std::includes(set.begin(), set.end(), subset.begin(), subset.end()) );	
}

int main()
{
	Timer t;
	t.reset();
	std::vector<int> s1{1,2,3};
	std::vector<int> s2{};
	std::vector<int> s3{2,3};
	std::vector<int> s4{2,4};

	std::vector<int> g1{1,2,3};
	std::vector<int> g2{3};
	std::vector<int> g3{2,3};
	std::vector<int> g4{2,4};

	std::vector<int> intersect_s = label_intersection(s1, s2, s3, s4);
	std::vector<int> intersect_g = label_intersection(g1, g2, g3, g4);
	std::vector<int> union_sg = label_union(intersect_s, intersect_g);


//=======================================================================================
	// create a sample labelmap which is created based on label{1,2,3}
	std::vector<std::vector<int>> labelCombinations;
	labelCombinations.push_back(std::vector<int>{});
	labelCombinations.push_back(std::vector<int>{3});
	labelCombinations.push_back(std::vector<int>{2});
	labelCombinations.push_back(std::vector<int>{2,3});
	labelCombinations.push_back(std::vector<int>{1});
	labelCombinations.push_back(std::vector<int>{1,3});
	labelCombinations.push_back(std::vector<int>{1,2});
	labelCombinations.push_back(std::vector<int>{1,2,3});
	std::vector<double> weightCombinations;
	weightCombinations.push_back(0.0);
	weightCombinations.push_back(0.1);
	weightCombinations.push_back(0.3);
	weightCombinations.push_back(0.4);
	weightCombinations.push_back(0.6);
	weightCombinations.push_back(0.7);
	weightCombinations.push_back(0.9);
	weightCombinations.push_back(1.0);

	// now zip two combinations together
	std::map<std::vector<int>, double> m;
	for (int i=0; i < labelCombinations.size(); ++i)
	{
		m[labelCombinations[i]] = weightCombinations[i];
	}

	// now come to sorted labelMap (VERIFIED)
	std::set<std::pair<std::vector<int>, double>, Comparator> labelMap;
	labelMap = std::set<std::pair<std::vector<int>, double>, Comparator>(
		m.begin(), m.end(), compFunctor);
//==========================================================================================
	// Final step to extract sub labelmap from labelmap based on union_sg
	std::map<std::vector<int>, double> sub_m;
	for (auto const &e : labelMap)
	{
		if (check_subset(e.first, union_sg))
		{
			sub_m[e.first] = e.second;
		}
	}
	std::set<std::pair<std::vector<int>, double>, Comparator> subLabelMap;
	subLabelMap = std::set<std::pair<std::vector<int>, double>, Comparator>(
		sub_m.begin(), sub_m.end(), compFunctor);


	// print the subLabelMap
	printf("*********sub labelMap************\n");
	//Iterate over the set you just come up with
	for (auto const &e : subLabelMap)
	{
		std::cout << "<";
		for (auto const &l : e.first)
		{
			std::cout << l << ",";
		}
		std::cout << "> :\t\t";
		std::cout << e.second << std::endl;
	}		

	std::cout << t.elapsed() << "\n";
	return 0;

}





