/*This cpp files now do 20 experiments on greedy and fixedLabel 
//for each setting (nLabels, labelCoverage, gridSize)*/

#include "LabeledGraph.hpp"
#include "PmcrGreedySolver.hpp"
#include "FixedLabelSolver.hpp"
#include "Timer.hpp"

#include <iostream>
#include <fstream>
#include <cstdlib> // for std::srand()
#include <ctime>


int main()
{
	Timer t;
	std::srand(std::time(0));
	int nExperiments = 20;

	// first do experiment on the computation time/survivability vs label coverage //
	// gridSize = 15*15, nLabels = 5
	// labelCoverage option: 30%, 40%, 50%, 60%
	//////////////////////////////////////////////////////////////////////////////////
	std::vector<double> labelCoverage{30, 40, 50, 60};

	// write into a txt file
	std::ofstream file_("labelCoverage_performance.txt");

	// experiment on each labelCoverage
	for (auto const &lc : labelCoverage)
	{
		double time_G = 0.0;
		double solution_G = 0.0;
		double time_F = 0.0;
		double solution_F = 0.0;
		for (int i=0; i < nExperiments; i++)
		{
			// generate a graph
			LabeledGraph_t g(15, 15, 5, lc/100.0);
			int start = random_generate_integer(0, 15*15-1);
			int goal = random_generate_integer(0, 15*15-1);
			while (start == goal)
			{
				int start = random_generate_integer(0, 15*15-1);
				int goal = random_generate_integer(0, 15*15-1);
			}
			//g.write_graph();//

			t.reset();
			std::cout << "----------start the fixedLabel search-------------\n";
			FixedLabelSolver_t fixedlabel_solver(g, start, goal);
			fixedlabel_solver.fixedLabel_search();
			time_F += t.elapsed();
			solution_F += (1 - fixedlabel_solver.getCurrentWeight());
			t.reset();
			std::cout << "----------start the greedy search-------------\n";
			PmcrGreedySolver_t pmcr_solver(g, start, goal);
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
		file_ << lc << " " << time_G << " " << solution_G << " " 
				<< time_F << " " << solution_F << "\n";
	}
	//////////////////////////////////////////////////////////////////////////////////


	// second do experiment on the computation time/survivability vs grid size //
	// nLabels = 5, labelCoverage = 40%
	// gridSize option: 10, 15, 20, 25
	///////////////////////////////////////////////////////////////////////////////////
	std::vector<int> gridSize{10, 15, 20, 25};

	// write into a txt file
	std::ofstream file1_("gridSize_performance.txt");

	// experiment on each gridSize
	for (auto const &gs : gridSize)
	{
		double time_G = 0.0;
		double solution_G = 0.0;
		double time_F = 0.0;
		double solution_F = 0.0;
		for (int i=0; i < nExperiments; i++)
		{
			// generate a graph
			LabeledGraph_t g(gs, gs, 5, 0.4);
			int start = random_generate_integer(0, gs*gs-1);
			int goal = random_generate_integer(0, gs*gs-1);
			while (start == goal)
			{
				int start = random_generate_integer(0, gs*gs-1);
				int goal = random_generate_integer(0, gs*gs-1);
			}
			//g.write_graph();//

			t.reset();
			std::cout << "----------start the fixedLabel search-------------\n";
			FixedLabelSolver_t fixedlabel_solver(g, start, goal);
			fixedlabel_solver.fixedLabel_search();
			time_F += t.elapsed();
			solution_F += (1 - fixedlabel_solver.getCurrentWeight());
			t.reset();
			std::cout << "----------start the greedy search-------------\n";
			PmcrGreedySolver_t pmcr_solver(g, start, goal);
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
		file1_ << gs << " " << time_G << " " << solution_G << " " 
				<< time_F << " " << solution_F << "\n";		
	}
	//////////////////////////////////////////////////////////////////////////////////



	// third do experiment on the computation time/survivability vs nLabels //
	// gridSize = 15*15, labelCoverage = 40%
	// nLabels option: 4, 5, 6, 7
	///////////////////////////////////////////////////////////////////////////////////
	std::vector<int> nLabels{4, 5, 6, 7};

	// write into a txt file
	std::ofstream file2_("nLabels_performance.txt");

	// experiment on each gridSize
	for (auto const &nl : nLabels)
	{
		double time_G = 0.0;
		double solution_G = 0.0;
		double time_F = 0.0;
		double solution_F = 0.0;
		for (int i=0; i < nExperiments; i++)
		{
			// generate a graph
			LabeledGraph_t g(15, 15, nl, 0.4);
			int start = random_generate_integer(0, 15*15-1);
			int goal = random_generate_integer(0, 15*15-1);
			while (start == goal)
			{
				int start = random_generate_integer(0, 15*15-1);
				int goal = random_generate_integer(0, 15*15-1);
			}
			//g.write_graph();//

			t.reset();
			std::cout << "----------start the fixedLabel search-------------\n";
			FixedLabelSolver_t fixedlabel_solver(g, start, goal);
			fixedlabel_solver.fixedLabel_search();
			time_F += t.elapsed();
			solution_F += (1 - fixedlabel_solver.getCurrentWeight());
			t.reset();
			std::cout << "----------start the greedy search-------------\n";
			PmcrGreedySolver_t pmcr_solver(g, start, goal);
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
		file2_ << nl << " " << time_G << " " << solution_G << " " 
				<< time_F << " " << solution_F << "\n";		
	}
	//////////////////////////////////////////////////////////////////////////////////


	return 0;

}


// int main()
// {
// 	std::srand(static_cast<unsigned int>(std::time(nullptr)));

// 	LabeledGraph_t g(20, 20, 5);
// 	int start = 0;
// 	int goal = 399;
// 	g.write_graph();

// 	Timer t;
// 	std::cout << "----------start the fixedLabel search-------------\n";
// 	FixedLabelSolver_t fixedlabel_solver(g, start, goal);
// 	fixedlabel_solver.fixedLabel_search();
// 	std::cout << "\nTimer elapsed: " << t.elapsed() << " seconds\n";
// 	fixedlabel_solver.write_solution();

// 	t.reset();
// 	std::cout << "----------start the greedy search-------------\n";
// 	PmcrGreedySolver_t pmcr_solver(g, start, goal);
// 	pmcr_solver.greedy_search();
// 	std::cout << "\nTimer elapsed: " << t.elapsed() << " seconds\n";
// 	pmcr_solver.write_solution();

// 	return 0;

// }