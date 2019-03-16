/*This cpp files now do 20 experiments on greedy and exact search
//to test their performance (computation time, solution) with regard to #labels*/

// #include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "PmcrExactSolver.hpp"
#include "Timer.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand()
#include <ctime>
#include <string> // std::string, std::to_string
#include <vector>

int main(int argc, char** argv)
{
	Timer t;
	std::srand(std::time(0));
	int nExperiments = 20;

	// default setting
	int g_row = 100;
	int g_col = 100;

	std::string folder_dir(argv[1]);
	// std::string file_dir1 = "./" + folder_dir + "/labelCoverage_performance.txt";
	// std::string file_dir2 = "./" + folder_dir + "/gridSize_performance.txt";
	std::string file_dir3 = "./" + folder_dir + "/exact_greedy_nPoses_performance.txt";


	// third do experiment on the computation time/survivability vs nLabels //
	//gridSize = 100*100
	// nObstacles = 20,50,100,150,200,250,300
	// nPoses = 100, 250, 500, 750, 1000, 1250, 1500
	///////////////////////////////////////////////////////////////////////////////////
	std::vector<int> nObstacles{10, 15, 20, 25, 30};
	int nPosesPerObs = 5;
	std::vector<int> nPoses{50, 75, 100, 125, 150};

	// write into a txt file
	std::ofstream file_3(file_dir3);

	if (file_3.is_open())
	{
		// experiment on each gridSize
		for (auto const &nP : nPoses)
		{
			double time_G = 0.0;
			double solution_G = 0.0;
			double time_E = 0.0;
			double solution_E = 0.0;
			// double time_Gr = 0.0;
			// double solution_Gr = 0.0;
			std::vector<int> nP_PerObs(nP/nPosesPerObs, nPosesPerObs);
			for (int i=0; i < nExperiments; i++)
			{
				std::cout << "***************" << nP << ":" << i << "**************\n\n";
				// generate a graph
				ConnectedGraph_t g(g_row, g_col, nP_PerObs);
				//g.write_graph();//

				std::cout << "********************************************* \n";
				std::cout << "----------start the greedy search-------------\n";
				t.reset();
				PmcrGreedySolver_t pmcr_greedy_solver(g);
				pmcr_greedy_solver.greedy_search(g);
				time_G += t.elapsed();
				std::cout << "Greedy time: " << t.elapsed() << " seconds.\n\n";
				solution_G += pmcr_greedy_solver.getCurrentSurvival();

				std::cout << "*********************************************\n";
				std::cout << "----------start the exact search-------------\n";
				t.reset();
				PmcrExactSolver_t pmcr_exact_solver(g);
				pmcr_exact_solver.exact_search(g);
				time_E += t.elapsed();
				std::cout << "Exact time: " << t.elapsed() << " seconds.\n\n";
				solution_E += pmcr_exact_solver.getCurrentSurvival();
			}
			// calculate the average time and survivability
			time_G /= nExperiments;	
			solution_G /= nExperiments;
			time_E /= nExperiments;
			solution_E /= nExperiments;
			// time_Gr /= nExperiments;
			// solution_Gr /= nExperiments;
			// write your results into the file
			file_3 << nP << " " << time_G << " " << solution_G << " " 
					<< time_E << " " << solution_E << "\n";		
		}
		file_3 << "\n";
		file_3.close();		
	}
	//////////////////////////////////////////////////////////////////////////////////

	return 0;

}


