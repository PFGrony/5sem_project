#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "opencv2/opencv.hpp"
#include <vector>
#include <deque>
#include <cmath>
#include <string>

struct node
{
    int x;
    int y;
    double g=0;//distance;
    double h=0;//heuristicDistance;
    double f=0;
};

struct compareHeuristic
{
	bool operator()(const node& a, const node& b)
	{
		return a.h > b.h;
	}
};

struct compareCost
{
	bool operator()(const node& a, const node& b)
	{
		return a.f > b.f;
	}
};

struct ppPair
{
    int x;
    int y;
};


class pathPlanner
{
public:
    explicit pathPlanner(std::string path);
    cv::Mat getMap()
    {
        return map;
    }
    cv::Mat getMapCopy()
    {
        return mapCopy;
    }

    std::deque<ppPair> AStarPlan(ppPair start,ppPair goal);
    std::deque<ppPair> BFSPlan(ppPair start,ppPair goal);
    std::deque<ppPair> GBFSPlan(ppPair start, ppPair goal);
    void drawBrushfire();
    void drawPath();

    double getPathLength();
    void brushfirePoint(ppPair start);
    void viewPath();

private:
	//Init
    cv::Mat map;
    cv::Mat grayMap;
    cv::Mat mapCopy;
    cv::Mat brushfire;
    double **intMap;
    double **cameFromMap;
    bool drawedBrushfire=false;

    std::deque<ppPair> routelist;
    std::vector<std::vector<ppPair>> cameFrom;

    //Methods
    std::deque<ppPair> getPath(ppPair start, ppPair goal);
    void AStar(ppPair start, ppPair goal);
    void BFS(ppPair start, ppPair goal);
    void GBFS(ppPair start, ppPair goal);
    void pairToNode(ppPair var1, node &var2);

};

#endif // PATHPLANNER_H
