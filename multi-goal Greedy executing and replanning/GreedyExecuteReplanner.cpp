#include "GreedyExecuteReplanner.hpp"
#include "Timer.hpp"

GreedyExecuteReplanner_t::GreedyExecuteReplanner_t(ConnectedGraph_t &g, std::vector<int> currentPath)
{
	Timer t;
	m_nReplan = 0;
	m_ExecutionTime = 0.0;
	m_pathLength = 0;
	m_isDoomed = false;

	// initially the 6 variables are the same as the graph problem
	// never change during the replanning
	m_nlabelsPerObs = g.getnLabelsPerObs();
	m_targetObs = g.getmTargetObs();

	// changing during the replanning
	m_start = g.getmStart();
	m_goalSetD = g.getmGoalSet();
	m_targetPosesD = g.getmTargetPoses();
	m_labelWeights = g.getLabelWeights();
	m_path = currentPath;

	// since the start of the original path is guaranteed to be collision free
	// directly add it into the m_ExecutedPath
	m_ExecutedPath.push_back(m_path[m_path.size()-1]);

	t.reset();
	// // first execution
	// m_needReplan = execute(g);

	while (execute(g))
	{
		m_nReplan++;
		std::cout << "\n#Replan: " << m_nReplan << "\n";
		// create a new gsolver(with updated information) and perform a new planning/search
		std::cout << "Replan:: start: " << m_start << "\n";
		std::cout << "Replan:: goalSetD: \n";
		std::cout << "<";
		for (auto const &gs : m_goalSetD)
		{
			std::cout << gs << " ";
		}
		std::cout << ">\n";
		std::cout << "Replan:: targetPoseD: \n";
		std::cout << "<";
		for (auto const &tp : m_targetPosesD)
		{
			std::cout << tp << " ";
		}
		std::cout << ">\n";
		for (int ii=0; ii < m_labelWeights.size(); ii++)
		{
			std::cout << ii << ": " << m_labelWeights[ii].first
												<< "\t" << m_labelWeights[ii].second << "\n";
		}
		PmcrGreedySolver_t gsolver1(g, m_start, m_goalSetD, m_targetPosesD, m_labelWeights);
		gsolver1.greedy_search(g);

		// check if the ground truth is solvable
		if (gsolver1.getIsSolvable() == false) 
		{
			m_isDoomed = true;
			break;
		}
		else
		{
			m_path = gsolver1.getOptimalPath();
		}

	}
	// You are reaching here either
	// (1)The problem is solved (m_needReplan=false)
	// (2) It turns out to be a doomed ground truth
	std::cout << "Jump out of the replan loop\n";
	if (!m_isDoomed)
	{
		m_ExecutionTime = t.elapsed();
		m_pathLength = m_ExecutedPath.size();
		std::cout << "The ground truth is not doomed\n";
	}
	std::cout << "Finishing replanning\n";

}

bool GreedyExecuteReplanner_t::execute(ConnectedGraph_t &g)
{
	std::cout << "Start executing the path\n";
	bool needReplan = false;
	std::cout << "Initially needReplan: " << needReplan << "\n";
	// This function will execute any given path
	// But it is not a one-time path execution. Every time before it moves
	// it checks whether next move is safe (not hitting true obstacles).
	// If the next step is safe, execute it.
	// Otherwise, immediately stop and update the map (graph problem)
	std::vector<int> currentEdgeLabels;
	bool isSafe_transition;

	for (int ii = m_path.size()-1; ii >= 1; ii--)
	{
		// check the current transition is safe or not
		currentEdgeLabels = g.getEdgeLabels(m_path[ii], m_path[ii-1]);
		isSafe_transition = updateMap(g, currentEdgeLabels);
		if (isSafe_transition)
		{
			// It's a collision free transition, move!
			m_ExecutedPath.push_back(m_path[ii-1]);
		}
		else
		{
			// You are encountering a true obstacle, need to replan the path
			// Before you do that, update the start location as the place you stop
			updateStart(m_path[ii]);
			needReplan = true;
			std::cout << "Encountering true obstacles!\n";
			std::cout << "Need Replan: " << needReplan << "\n";
			return needReplan;
		}
	}
	// You are reaching here since you finish the path without collision
	// But it doesn't neccessarily mean you don't need replan
	// If you reach a goal which is not a true goal, you still need to replan
	std::cout << "reaching the destination: " << m_path[0] << "\n";
	std::cout << "The true goal is: " << g.getTrueGoal() << "\n";
	if (m_path[0] != g.getTrueGoal())
	{
		std::cout << "Safely reaching. But it is not a true goal.\n";
		updateStart(m_path[0]);
		// since it is not the true goal, we also know that pose it affiliates to
		// is not a true pose
		for (int pp = 0; pp < m_goalSetD.size(); pp++)
		{
			if (m_goalSetD[pp] == m_path[0])
			{
				updateLabelWeights(m_targetPosesD[pp], false);
				break;
			}
		}
		needReplan = true;
		std::cout << "Need Replan: " << needReplan << "\n";
	}

	return needReplan;
}


bool GreedyExecuteReplanner_t::updateMap(ConnectedGraph_t &g, std::vector<int> &labels)
{
	bool isSafe = true;
	// check each label
	for (auto const &l : labels)
	{
		// if the label has been set to 0.0 (non-existent)
		// There is no meaning analyze it 
		if (getLabelWeight(l) == 0.0) { continue; }
		if (g.getTruePoses(l) == true) 
		{
			isSafe = false;
			updateLabelWeights(l, true);
		}
		else
		{
			updateLabelWeights(l, false);
		}
	}

	return isSafe;
}

void GreedyExecuteReplanner_t::updateLabelWeights(int label, bool mode)
{
	int temp_obs;

	if (mode == true)
	{
		// You know the pose is a true pose/label
		m_labelWeights[label].second = 1.0;
		// Then change the weights of other labels which share the same obstacle
		// with this label to be 0.0
		temp_obs = m_labelWeights[label].first;
		for (int cl=temp_obs*m_nlabelsPerObs; cl<(temp_obs+1)*m_nlabelsPerObs; cl++)
		{
			if (cl != label) { m_labelWeights[cl].second = 0.0; }
		}
		// If the obstacle the label belongs to is a target obstacle
		if (temp_obs == m_targetObs)
		{
			// prune m_goalSetD & m_targetPosesD
			for (int dd=0; dd < m_targetPosesD.size(); dd++)
			{
				if (m_targetPosesD[dd] == label)
				{
					m_targetPosesD = std::vector<int>(1, label);
					m_goalSetD = std::vector<int>(1, m_goalSetD[dd]);
				}
			}
		}
	}
	else
	{
		// You know the pose is a fake pose/label
		// The change the weight of the label to be 0.0 and meanwhile assign the origin weight
		// of the this label to other labels sharing the same obstacle with this label according
		// to their distribution
		temp_obs = m_labelWeights[label].first;
		for (int cl=temp_obs*m_nlabelsPerObs; cl<(temp_obs+1)*m_nlabelsPerObs; cl++)
		{
			if (cl != label)
			{
				m_labelWeights[cl].second = 
					m_labelWeights[cl].second + 
						m_labelWeights[label].second * m_labelWeights[cl].second / (1 - m_labelWeights[label].second);
			}
		}
		// Don't forget to set the weight for current fake label to be 0.0;
		m_labelWeights[label].second = 0.0;
		// If the obstacle the label belongs to is a target obstacle
		if (temp_obs == m_targetObs)
		{
			// prune m_goalSetD & m_targetPosesD
			for (int dd=0; dd < m_targetPosesD.size(); dd++)
			{
				if (m_targetPosesD[dd] == label)
				{
					// prune that label from m_targetPosesD
					m_targetPosesD.erase(m_targetPosesD.begin()+dd);
					// prune the corresponding goal from m_goalSetD as well
					m_goalSetD.erase(m_goalSetD.begin()+dd);
				}
			}
		}
	}
} 

GreedyExecuteReplanner_t::~GreedyExecuteReplanner_t() 
{
	std::cout << "Destroy the replanner.\n";
}