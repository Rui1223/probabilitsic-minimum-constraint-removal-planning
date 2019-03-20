#include "GreedyExecuteReplanner.hpp"
#include "Timer.hpp"

GreedyExecuteReplanner_t::GreedyExecuteReplanner_t(ConnectedGraph_t &g, PmcrGreedySolver_t originalGsolver)
{
	Timer t;
	m_nReplan = 0;
	m_ExecutionTime = 0.0;
	m_pathLength = 0;
	m_isDoomed = false;

	int start;
	std::vector<int> goalSetD;
	std::vector<int> targetPoseD;
	std::map<int, std::pair<int, double>> labelWeights;

	// since the start of the original path is guaranteed to be collision free
	// directly add it into the m_ExecutedPath
	std::vector<int> originalPath = originalGsolver.getOptimalPath();
	m_ExecutedPath.push_back(originalPath[originalPath.size()-1]);

	t.reset();
	// first execution
	m_needReplan = execute(g, originalGsolver, originalPath);
	start = originalGsolver.getmStart();
	goalSetD = originalGsolver.getGoalSetD();
	targetPoseD = originalGsolver.getTargetPoseD();
	labelWeights = originalGsolver.getLabelWeights();

	while (m_needReplan and m_nReplan < 20)
	{
		m_nReplan++;
		std::cout << "#Replan: " << m_nReplan << "\n\n";
		// create a new gsolver(with updated information) and perform a new planning/search
		std::cout << "Replan:: start: " << start << "\n";
		std::cout << "Replan:: goalSetD: \n";
		std::cout << "<";
		for (auto const &gs : goalSetD)
		{
			std::cout << gs << " ";
		}
		std::cout << ">\n";
		std::cout << "Replan:: targetPoseD: \n";
		std::cout << "<";
		for (auto const &tp : targetPoseD)
		{
			std::cout << tp << " ";
		}
		std::cout << ">\n";
		for (int ii=0; ii < labelWeights.size(); ii++)
		{
			std::cout << ii << ": " << labelWeights[ii].first
												<< "\t" << labelWeights[ii].second << "\n";
		}
		PmcrGreedySolver_t gsolver1(g, start, goalSetD, targetPoseD, labelWeights);
		gsolver1.greedy_search(g);

		// check if the replanning path is a deemed path
		// if it is, the ground truth should be discard
		if (gsolver1.getOptimalSurvival() == 0.0) 
		{
			m_isDoomed = true;
			break;
		}
		std::vector<int> currentPath = gsolver1.getOptimalPath();
		m_needReplan = execute(g, gsolver1, currentPath);

		start = gsolver1.getmStart();
		goalSetD = gsolver1.getGoalSetD();
		targetPoseD = gsolver1.getTargetPoseD();
		labelWeights = gsolver1.getLabelWeights();
	}
	// You are reaching here either
	// (1)The problem is solved (m_needReplan=false)
	// (2) It turns out to be a doomed ground truth
	std::cout << "Jump out of the replan loop\n";
	if (!m_isDoomed)
	{
		m_ExecutionTime = t.elapsed();
		m_pathLength = m_ExecutedPath.size();
	}
	std::cout << "Finishing replanning\n";

}

bool GreedyExecuteReplanner_t::execute(ConnectedGraph_t &g, 
						PmcrGreedySolver_t &gsolver, std::vector<int> &path)
{
	std::cout << "Start executing the path\n";
	bool needReplan = false;
	// This function will execute any given path
	// But it is not a one-time path execution. Every time before it moves
	// it checks whether next move is safe (not hitting true obstacles).
	// If the next step is safe, execute it.
	// Otherwise, immediately stop and update the map (graph problem)
	std::vector<int> currentEdgeLabels;
	bool isSafe_transition;

	for (int ii = path.size()-1; ii >= 1; ii--)
	{
		// check the current transition is safe or not
		currentEdgeLabels = g.getEdgeLabels(path[ii], path[ii-1]);
		isSafe_transition = updateMap(g, gsolver, currentEdgeLabels);
		if (isSafe_transition)
		{
			// It's a collision free transition, move!
			m_ExecutedPath.push_back(path[ii-1]);
		}
		else
		{
			// You are encountering a true obstacle, need to replan the path
			// Before you do that, update the start location as the place you stop
			gsolver.updateStart(path[ii]);
			needReplan = true;
			std::cout << "Need Replan: " << needReplan << "\n";
			return needReplan;
		}
	}
	// You are reaching here since you finish the path without collision
	// But it doesn't neccessarily mean you don't need replan
	// If you reach a goal which is not a true goal, you still need to replan
	std::cout << "reaching the destination: " << path[0] << "\n";
	std::cout << "The true goal is: " << g.getTrueGoal() << "\n";
	if (path[0] != g.getTrueGoal())
	{
		std::cout << "Safely reaching. But it is not a true goal.\n";
		gsolver.updateStart(path[0]);
		// since it is not the true goal, we also know that pose it affiliates to
		// is not a true pose
		std::vector<int> temp_goalSetD = gsolver.getGoalSetD();
		std::vector<int> temp_TargetPosesD = gsolver.getTargetPoseD();
		for (int pp = 0; pp < temp_goalSetD.size(); pp++)
		{
			if (temp_goalSetD[pp] == path[0])
			{
				gsolver.updateLabelWeights(temp_TargetPosesD[pp], false);
				break;
			}
		}
		needReplan = true;
		std::cout << "Need Replan: " << needReplan << "\n";
	}

	return needReplan;
}


bool GreedyExecuteReplanner_t::updateMap(ConnectedGraph_t &g, 
					PmcrGreedySolver_t &gsolver, std::vector<int> &labels)
{
	bool isSafe = true;
	// check each label
	for (auto const &l : labels)
	{
		// if the label has been set to 0.0 (non-existent)
		// There is no meaning analyze it 
		if (gsolver.getLabelWeight(l) == 0.0) { continue; }
		if (g.getTruePoses(l) == true) 
		{
			isSafe = false;
			gsolver.updateLabelWeights(l, true);
		}
		else
		{
			gsolver.updateLabelWeights(l, false);
		}
	}

	return isSafe;
}

GreedyExecuteReplanner_t::~GreedyExecuteReplanner_t() 
{
	std::cout << "Destroy the replanner.\n";
}