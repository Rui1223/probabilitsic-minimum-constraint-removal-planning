/*This cpp files now do 20 experiments on greedy and fixedLabel 
//for each setting (nLabels, labelCoverage, gridSize)*/

// #include "LabeledGraph.hpp"
#include "ConnectedGraph.hpp"
// #include "ConnectedNonOverlapGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "FixedLabelSolver.hpp"
// #include "GrowingTreeSolver.hpp"
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
	int g_row = 50;
	int g_col = 50;
	int g_nlabels = 15;
	int g_nlabels_unit = 3;
	std::vector<int> g_nlabelsPerObs(g_nlabels/g_nlabels_unit, g_nlabels_unit);
	double g_probPerLabel = 0.5;

	std::string folder_dir(argv[1]);
	std::string file_dir1 = "./" + folder_dir + "/labelCoverage_performance.txt";
	std::string file_dir2 = "./" + folder_dir + "/gridSize_performance.txt";
	std::string file_dir3 = "./" + folder_dir + "/nLabels_performance.txt";

	// first do experiment on the computation time/survivability vs label coverage //
	// gridSize = 50*50, g_nlabels = 15
	// labelCoverage option: 40%, 50%, 60%
	//////////////////////////////////////////////////////////////////////////////////
	std::vector<double> labelCoverage{40, 50, 60};

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
			// double time_Gr = 0.0;
			// double solution_Gr = 0.0;
			for (int i=0; i < nExperiments; i++)
			{
				std::cout << "***************" << lc << ":" << i << "**************\n";
				// generate a graph
				ConnectedGraph_t g(g_row, g_col, g_nlabelsPerObs, lc/100.0);
				int start = random_generate_integer(0, g_row*g_col-1);
				int goal = random_generate_integer(0, g_row*g_col-1);
				while (start == goal)
				{
					start = random_generate_integer(0, g_row*g_col-1);
					goal = random_generate_integer(0, g_row*g_col-1);
				}
				//g.write_graph();//

				std::cout << "----------start the fixedLabel search-------------\n";
				FixedLabelSolver_t fixedlabel_solver(g, start, goal);
				t.reset();
				fixedlabel_solver.fixedLabel_search();
				time_F += t.elapsed();
				solution_F += fixedlabel_solver.getCurrentSurvival();
				std::cout << "----------start the greedy search-------------\n";
				PmcrGreedySolver_t pmcr_solver(g, start, goal);
				t.reset();
				pmcr_solver.greedy_search();
				time_G += t.elapsed();
				solution_G += pmcr_solver.getCurrentSurvival();
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
	// g_nlabels = 15, g_probPerLabel = 0.5
	// gridSize option: 35, 50, 100
	///////////////////////////////////////////////////////////////////////////////////
	std::vector<int> gridSize{35, 50, 100};

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
			// double time_Gr = 0.0;
			// double solution_Gr = 0.0;
			for (int i=0; i < nExperiments; i++)
			{
				std::cout << "***************" << gs << ":" << i << "**************\n";
				// generate a graph
				ConnectedGraph_t g(gs, gs, g_nlabelsPerObs, g_probPerLabel);
				int start = random_generate_integer(0, gs*gs-1);
				int goal = random_generate_integer(0, gs*gs-1);
				while (start == goal)
				{
					start = random_generate_integer(0, gs*gs-1);
					goal = random_generate_integer(0, gs*gs-1);
				}
				//g.write_graph();//

				std::cout << "----------start the fixedLabel search-------------\n";
				FixedLabelSolver_t fixedlabel_solver(g, start, goal);
				t.reset();
				fixedlabel_solver.fixedLabel_search();
				time_F += t.elapsed();
				solution_F += fixedlabel_solver.getCurrentSurvival();
				std::cout << "----------start the greedy search-------------\n";
				PmcrGreedySolver_t pmcr_solver(g, start, goal);
				t.reset();
				pmcr_solver.greedy_search();
				time_G += t.elapsed();
				solution_G += pmcr_solver.getCurrentSurvival();
				// std::cout << "----------start the growingTree search-------------\n";
				// GrowingTreeSolver_t growingtree_solver(g, start, goal);
				// t.reset();
				// growingtree_solver.GrowingTreeSearch();
				// time_Gr += t.elapsed();
				// solution_Gr += (1 - growingtree_solver.getCurrentWeight());
			}
			// calculate the average time and survivability
			time_G /= nExperiments;	
			solution_G /= nExperiments;
			time_F /= nExperiments;
			solution_F /= nExperiments;
			// time_Gr /= nExperiments;
			// solution_Gr /= nExperiments;
			// write your results into the file
			file_2 << gs << " " << time_G << " " << solution_G << " " 
					<< time_F << " " << solution_F << "\n";		
		}
		file_2 << "\n";
		file_2.close();		
	}
	//////////////////////////////////////////////////////////////////////////////////



	// third do experiment on the computation time/survivability vs nLabels //
	//gridSize = 50*50, labelCoverage option: 50%
	// nLabels option: 9, 15, 21
	///////////////////////////////////////////////////////////////////////////////////
	std::vector<int> nLabels{9, 15, 21};

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
			// double time_Gr = 0.0;
			// double solution_Gr = 0.0;
			std::vector<int> nl_PerObs(nl/g_nlabels_unit, g_nlabels_unit);
			for (int i=0; i < nExperiments; i++)
			{
				std::cout << "***************" << nl << ":" << i << "**************\n";
				// generate a graph
				ConnectedGraph_t g(g_row, g_col, nl_PerObs, g_probPerLabel);
				int start = random_generate_integer(0, g_row*g_col-1);
				int goal = random_generate_integer(0, g_row*g_col-1);
				while (start == goal)
				{
					start = random_generate_integer(0, g_row*g_col-1);
					goal = random_generate_integer(0, g_row*g_col-1);
				}
				//g.write_graph();//


				std::cout << "----------start the fixedLabel search-------------\n";
				FixedLabelSolver_t fixedlabel_solver(g, start, goal);
				t.reset();
				fixedlabel_solver.fixedLabel_search();
				time_F += t.elapsed();
				solution_F += fixedlabel_solver.getCurrentSurvival();
				std::cout << "----------start the greedy search-------------\n";
				PmcrGreedySolver_t pmcr_solver(g, start, goal);
				t.reset();
				pmcr_solver.greedy_search();
				time_G += t.elapsed();
				solution_G += pmcr_solver.getCurrentSurvival();
				// std::cout << "----------start the growingTree search-------------\n";
				// GrowingTreeSolver_t growingtree_solver(g, start, goal);
				// t.reset();
				// growingtree_solver.GrowingTreeSearch();
				// time_Gr += t.elapsed();
				// solution_Gr += (1 - growingtree_solver.getCurrentWeight());
			}
			// calculate the average time and survivability
			time_G /= nExperiments;	
			solution_G /= nExperiments;
			time_F /= nExperiments;
			solution_F /= nExperiments;
			// time_Gr /= nExperiments;
			// solution_Gr /= nExperiments;
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


