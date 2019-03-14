/*The cpp file do several experiements on greedy ahnd fixedLabel search*/
/*save graphs and several corresponding solutions, respectively*/

#include "ConnectedGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "FixedLabelSolver.hpp"
#include "PmcrExactSolver.hpp"
#include "Timer.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand(), std::rand()
#include <ctime>
#include <string> // std::string, std::to_string

int main(int argc, char** argv)
{
	int nExperiments = 1;
	int row = 100;
	int col = 100;
	std::vector<int> nLabelsPerObs{4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4};
	double density = 0.6;
	Timer t;
	std::srand(std::time(0));
	double t1;

	std::string folder_dir(argv[1]);

	for (int ii = 0; ii < nExperiments; ii++)
	{
		std::string file_dir1 = "./" + folder_dir + "/graph" + std::to_string(ii) + ".txt";
		std::string file_dir2 = "./" + folder_dir + "/GreedySearch_solution" 
																+ std::to_string(ii) + ".txt";
		std::string file_dir3 = "./" + folder_dir + "/ExactSearch_solution" 
																+ std::to_string(ii) + ".txt";	
		// std::string file_dir4 = "./" + folder_dir + "/FixedLabel_solution" 
		// 													+ std::to_string(ii) + ".txt";

		// generate a grpah
		ConnectedGraph_t g(row, col, nLabelsPerObs, density);
		g.write_graph(file_dir1);
		std::cout << std::endl;

		// work out on solutions
		std::cout << "**********************************************\n";
		std::cout << "----------start the greedy search-------------\n";
		t.reset();
		PmcrGreedySolver_t pmcr_greedy_solver(g);
		pmcr_greedy_solver.greedy_search(g);
		t1 = t.elapsed();
		std::cout << "Greedy time: " << t1 << " seconds\n\n";
		pmcr_greedy_solver.write_solution(file_dir2, t1);

		std::cout << "**********************************************\n";
		std::cout << "----------start the exact search-------------\n";
		t.reset();
		PmcrExactSolver_t pmcr_exact_solver(g);
		pmcr_exact_solver.exact_search(g);
		t1 = t.elapsed();
		std::cout << "exact time: " << t1 << " seconds\n\n";
		pmcr_exact_solver.write_solution(file_dir3, t1);

		// std::cout << "**************************************************\n";
		// std::cout << "----------start the fixedLabel search-------------\n";
		// t.reset();
		// FixedLabelSolver_t fixedlabel_solver(g);
		// fixedlabel_solver.fixedLabel_search(g);
		// t1 = t.elapsed();
		// std::cout << "FixedLabel time: " << t1 << " seconds\n\n";
		// fixedlabel_solver.write_solution(file_dir4, t1);

	}

	return 0;
}