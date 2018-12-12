/*test whether the powerSet integration works*/
#include "LabeledGraph.hpp"

#include <cstdio>
#include <iostream>
#include <queue>



int main()
{
	// Problem input
	LabeledGraph_t g(2, 6);
	std::cout << "Now print labelMap\n";
	g.cal_labelMap();
	g.print_labelMap();

	return 0;

}