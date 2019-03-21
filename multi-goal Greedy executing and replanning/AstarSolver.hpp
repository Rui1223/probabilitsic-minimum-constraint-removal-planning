#ifndef ASTARSOLVER_H
#define ASTARSOLVER_H

#include <queue>
#include <cstring>

#include "ConnectedGraph.hpp"


struct AstarNode_t
{
	int m_id;
	int m_H;
	int m_F;
	AstarNode_t *m_parent;
	AstarNode_t(int id, int H, int F, AstarNode_t *parent)
	{
		m_id = id;
		m_H = H;
		m_F = F;
		m_parent = parent;
	}
};

struct AstarNode_comparison
{
	bool operator()(const AstarNode_t* a, const AstarNode_t* b) 
	{
		if (a->m_F == b->m_F)
		{
			return (a->m_H) > (b->m_H);
		}
		return (a->m_F) > (b->m_F);
	}	
};

class AstarSolver_t
{
	int m_col;
	int m_targetObs; //obs idx for target
	int m_nlabelsPerObs;
	int m_nObstacles;

	std::vector<int> m_goalSet;
	std::vector<int> m_targetPoses;

	std::vector<int> m_path;
	std::priority_queue<AstarNode_t*, std::vector<AstarNode_t*>, AstarNode_comparison> m_open;
	std::vector<AstarNode_t*> m_closed;
	std::vector<bool> m_expanded;
	std::vector<int> m_G;

	// replanning things
	int m_start; // the id of the start node
	std::vector<int> m_goalSetD;
	std::vector<int> m_targetPosesD;
	std::map<int, std::pair<int, double>> m_labelWeights;

	bool m_solvable;

public:
	AstarSolver_t(ConnectedGraph_t &g, int start, std::vector<int> goalSetD, 
			std::vector<int> targetPosesD, std::map<int, std::pair<int, double>> labelWeights);
	~AstarSolver_t();
	void Astar_search(ConnectedGraph_t &g);
	void back_track_path();
	// The function to compute the h value of a node(indx) for a given goal
	int computeH(int indx);
	bool collision_check(ConnectedGraph_t &g, std::vector<int> &edgeLabels);
	void write_solution(std::string file_dir, double t);
	void print_closedList();

	// getters
	int getmStart() { return m_start; }

};

#endif