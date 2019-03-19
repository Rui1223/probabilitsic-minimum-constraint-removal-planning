#include "GreedyExecuteReplanner.hpp"
#include "Timer.hpp"

ExecuteReplanner_t::ExecuteReplanner_t(ConnectedGraph_t &g, PmcrGreedySolver_t &gsolver)
{
	Timer t;
	int m_nReplan = 0;
	double m_ExecutionTime = 0.0;
	int m_pathLength = 0;
	// since the start of the original path is guaranteed to be collision free
	// directly add it into the m_ExecutedPath
	std::vector<int> currentPath = gsolver.getOptimalPath();
	m_ExecutedPath.push_back(currentPath[currentPath.size()-1]);
	t.reset()
	while (GreedyExecutor.execute(g, gsolver, currentPath))
	{
		m_nReplan++;
		gsolver.greedy_search(g);
		currentPath = gsolver.getOptimalPath();
	}
	// You are reaching here since the problem is solved (NeedReplan=false)
	m_ExecutionTime = t.elapsed();
	m_pathLength = m_ExecutedPath.size();
}

bool ExecuteReplanner_t::execute(ConnectedGraph_t &g, 
						PmcrGreedySolver_t &gsolver, std::vector<int> &path)
{
	bool NeedReplan = false;
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
			g.updateStart(path[ii]);
			NeedReplan = true;
			return NeedReplan;
		}
	}
	// You are reaching here since you finish the path without collision
	// But it doesn't neccessarily mean you don't need replan
	// If you reach a goal which is not a true goal, you still need to replan
	if (path[0] != g.getTrueGoal())
	{
		g.updateStart(path[0]);
		NeedReplan = true;
	}

	return NeedReplan;

}


bool ExecuteReplanner_t::updateMap(ConnectedGraph_t &g, 
					PmcrGreedySolver_t &gsolver, std::vector<int> &labels)
{
	bool isSafe = true;
	// check each label
	for (auto const &l : labels)
	{
		if (g.getTruePoses[l] == true) 
		{
			isSafe = false;
			gsolver.updateLabelWeights(l, true);
		}
		else
		{
			g.updateLabelWeights(l, false);
		}
	}

	return isSafe;
}