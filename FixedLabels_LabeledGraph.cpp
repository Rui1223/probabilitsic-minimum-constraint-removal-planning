#include "FixedLabels_LabeledGraph.hpp"

// Constructor 
FixedLabels_LabeledGraph_t::FixedLabels_LabeledGraph_t(int row, int col)
	: LabeledGraph_t(row, col)
{
	std::cout << "The same construction procedure as superclass LabeledGraph\n";
}

int main()
{
	FixedLabels_LabeledGraph_t g(2, 6);
	return 0;
}