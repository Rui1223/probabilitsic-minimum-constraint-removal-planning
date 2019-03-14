/*This cpp file now do 15 experiments in terms of number of labels 
to test how computation time scales*/

#include "ConnectedGraph.hpp"
#include "FixedLabelSolver.hpp"
#include "Timer.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <string> // std::string, std::to_string


int main(int argc, char** argv)
{
	Timer t;
	std::srand(std::time(0));
	int nExperiments = 15;

	// default setting
	int g_row = 100;
	int g_col = 100;
	// int g_nlabels = 15;
	int g_nlabels_unit = 3;
	// std::vector<int> g_nlabelsPerObs(g_nlabels/g_nlabels_unit, g_nlabels_unit);
	// double g_density = 0.6;

	std::string folder_dir(argv[1]);
	// std::string file_dir1 = "./" + folder_dir + "/labelCoverage_performance.txt";
	// std::string file_dir2 = "./" + folder_dir + "/gridSize_performance.txt";
	std::string file_dir3 = "./" + folder_dir + "/fixedLabel_performance.txt";


	//gridSize = 100*100, labelCoverage: 60%
	// nLabels option: 3, 6, 9, 12, 15, 18, 21
	///////////////////////////////////////////////////////////////////////////////////
	std::vector<int> nLabels{3, 6, 9, 12, 15, 18, 21};

	// write into a txt file
	std::ofstream file_3(file_dir3);

	if (file_3.is_open())
	{
		// experiment on each gridSize
		for (auto const &nl : nLabels)
		{
			double time_F = 0.0;
			// double time_Gr = 0.0;
			// double solution_Gr = 0.0;
			std::vector<int> nl_PerObs(nl/g_nlabels_unit, g_nlabels_unit);
			for (int i=0; i < nExperiments; i++)
			{
				std::cout << "***************" << nl << ":" << i << "**************\n";
				// generate a graph
				ConnectedGraph_t g(g_row, g_col, nl_PerObs);

				std::cout << "----------start the fixedLabel search-------------\n";
				t.reset();
				FixedLabelSolver_t fixedlabel_solver(g);
				fixedlabel_solver.fixedLabel_search(g);
				std::cout << "Total time is: " << t.elapsed() << " seconds.\n";
				time_F += t.elapsed();
			}
			// calculate the average time for each case
			time_F /= nExperiments;
			// write your results (computation time) into the file
			file_3 << nl << " " << time_F << " " << "\n";		
		}
		file_3 << "\n";
		file_3.close();		
	}
	//////////////////////////////////////////////////////////////////////////////////


	return 0;

}