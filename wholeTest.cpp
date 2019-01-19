/*This cpp files now do 20 experiments on greedy and fixedLabel 
//for each setting (nLabels, labelCoverage, gridSize)*/

#include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
#include "ConnectedNonOverlapGraph.hpp"
#include "PmcrGreedySolver.hpp"
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
	int nExperiments = 20;

	std::string folder_dir(argv[1]);
	std::string file_dir1 = "./" + folder_dir + "/labelCoverage_performance.txt";
	std::string file_dir2 = "./" + folder_dir + "/gridSize_performance.txt";
	std::string file_dir3 = "./" + folder_dir + "/nLabels_performance.txt";

	// first do experiment on the computation time/survivability vs label coverage //
	// gridSize = 25*25, nLabels = 4
	// labelCoverage option: 20%, 30%, 40%
	//////////////////////////////////////////////////////////////////////////////////
	std::vector<double> labelCoverage{20, 30, 40};

	// write into a txt file
	std::ofstream file_1(file_dir1);

	if (file_1.is_open())
	{
		// experiment on each labelCoverage
		for (auto const &lc : labelCoverage)
		{
			double time_G = 0.0;
			double solution_G = 0.0;
			double time_F = 0.0;
			double solution_F = 0.0;
			for (int i=0; i < nExperiments; i++)
			{
				std::cout << "***************" << lc << ":" << i << "**************\n";
				// generate a graph
				ConnectedGraph_t g(25, 25, 4, lc/100.0);
				int start = random_generate_integer(0, 25*25-1);
				int goal = random_generate_integer(0, 25*25-1);
				while (start == goal)
				{
					start = random_generate_integer(0, 25*25-1);
					goal = random_generate_integer(0, 25*25-1);
				}
				//g.write_graph();//

				std::cout << "----------start the fixedLabel search-------------\n";
				FixedLabelSolver_t fixedlabel_solver(g, start, goal);
				t.reset();
				fixedlabel_solver.fixedLabel_search();
				time_F += t.elapsed();
				solution_F += (1 - fixedlabel_solver.getCurrentWeight());
				std::cout << "----------start the greedy search-------------\n";
				PmcrGreedySolver_t pmcr_solver(g, start, goal);
				t.reset();
				pmcr_solver.greedy_search();
				time_G += t.elapsed();
				solution_G += (1 - pmcr_solver.getCurrentWeight());
			}
			// calculate the average time and survivability
			time_G /= nExperiments;	
			solution_G /= nExperiments;
			time_F /= nExperiments;
			solution_F /= nExperiments;
			// write your results into the file
			file_1 << lc << " " << time_G << " " << solution_G << " " 
					<< time_F << " " << solution_F << "\n";
		}
		file_1 << "\n";
		file_1.close();		
	}
	//////////////////////////////////////////////////////////////////////////////////


	// second do experiment on the computation time/survivability vs grid size //
	// nLabels = 4, labelCoverage = 30%
	// gridSize option: 10, 15, 20, 25
	///////////////////////////////////////////////////////////////////////////////////
	std::vector<int> gridSize{15, 25, 35};

	// write into a txt file
	std::ofstream file_2(file_dir2);

	if (file_2.is_open())
	{
		// experiment on each gridSize
		for (auto const &gs : gridSize)
		{
			double time_G = 0.0;
			double solution_G = 0.0;
			double time_F = 0.0;
			double solution_F = 0.0;
			for (int i=0; i < nExperiments; i++)
			{
				std::cout << "***************" << gs << ":" << i << "**************\n";
				// generate a graph
				ConnectedGraph_t g(gs, gs, 4, 0.3);
				int start = random_generate_integer(0, gs*gs-1);
				int goal = random_generate_integer(0, gs*gs-1);
				while (start == goal)
				{
					int start = random_generate_integer(0, gs*gs-1);
					int goal = random_generate_integer(0, gs*gs-1);
				}
				//g.write_graph();//

				std::cout << "----------start the fixedLabel search-------------\n";
				FixedLabelSolver_t fixedlabel_solver(g, start, goal);
				t.reset();
				fixedlabel_solver.fixedLabel_search();
				time_F += t.elapsed();
				solution_F += (1 - fixedlabel_solver.getCurrentWeight());
				std::cout << "----------start the greedy search-------------\n";
				PmcrGreedySolver_t pmcr_solver(g, start, goal);
				t.reset();
				pmcr_solver.greedy_search();
				time_G += t.elapsed();
				solution_G += (1 - pmcr_solver.getCurrentWeight());
			}
			// calculate the average time and survivability
			time_G /= nExperiments;	
			solution_G /= nExperiments;
			time_F /= nExperiments;
			solution_F /= nExperiments;
			// write your results into the file
			file_2 << gs << " " << time_G << " " << solution_G << " " 
					<< time_F << " " << solution_F << "\n";		
		}
		file_2 << "\n";
		file_2.close();		
	}
	//////////////////////////////////////////////////////////////////////////////////



	// third do experiment on the computation time/survivability vs nLabels //
	// gridSize = 25*25, labelCoverage = 30%
	// nLabels option: 4, 5, 6, 7
	///////////////////////////////////////////////////////////////////////////////////
	std::vector<int> nLabels{2, 4, 8};

	// write into a txt file
	std::ofstream file_3(file_dir3);

	if (file_3.is_open())
	{
		// experiment on each gridSize
		for (auto const &nl : nLabels)
		{
			double time_G = 0.0;
			double solution_G = 0.0;
			double time_F = 0.0;
			double solution_F = 0.0;
			for (int i=0; i < nExperiments; i++)
			{
				std::cout << "***************" << nl << ":" << i << "**************\n";
				// generate a graph
				ConnectedGraph_t g(25, 25, nl, 0.3);
				int start = random_generate_integer(0, 25*25-1);
				int goal = random_generate_integer(0, 25*25-1);
				while (start == goal)
				{
					int start = random_generate_integer(0, 25*25-1);
					int goal = random_generate_integer(0, 25*25-1);
				}
				//g.write_graph();//


				std::cout << "----------start the fixedLabel search-------------\n";
				FixedLabelSolver_t fixedlabel_solver(g, start, goal);
				t.reset();
				fixedlabel_solver.fixedLabel_search();
				time_F += t.elapsed();
				solution_F += (1 - fixedlabel_solver.getCurrentWeight());
				std::cout << "----------start the greedy search-------------\n";
				PmcrGreedySolver_t pmcr_solver(g, start, goal);
				t.reset();
				pmcr_solver.greedy_search();
				time_G += t.elapsed();
				solution_G += (1 - pmcr_solver.getCurrentWeight());
			}
			// calculate the average time and survivability
			time_G /= nExperiments;	
			solution_G /= nExperiments;
			time_F /= nExperiments;
			solution_F /= nExperiments;
			// write your results into the file
			file_3 << nl << " " << time_G << " " << solution_G << " " 
					<< time_F << " " << solution_F << "\n";		
		}
		file_3 << "\n";
		file_3.close();		
	}
	//////////////////////////////////////////////////////////////////////////////////


	return 0;

}


