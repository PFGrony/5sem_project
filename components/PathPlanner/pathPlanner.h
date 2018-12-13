#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "opencv2/opencv.hpp"
#include <chrono>

#include <array>
#include <vector>
#include <deque>
#include <cmath>
#include <string>

//Big Map
#define ROW  80
#define COL  120
////Small Map
//#define ROW  15
//#define COL  20

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

struct compare
{
	bool operator()(const int& a, const int& b)
	{
		return a > b;
	}
};

struct pair
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

    std::deque<pair> AStarPlan(pair start,pair goal);
    std::deque<pair> BFSPlan(pair start,pair goal);
    std::deque<pair> GBFSPlan(pair start, pair goal);
    void drawBrushfire();
    void drawPath();

    double getPathLength();
    void brushfirePoint(pair start);
    void viewPath();

private:
	//Init
    cv::Mat map;
    cv::Mat grayMap;
    cv::Mat mapCopy;
    cv::Mat brushfire;
    double **intMap;//[ROW][COL];
    double **cameFromMap;//[ROW][COL];
    bool drawedBrushfire=false;

    std::deque<pair> routelist;
	std::array<std::array<pair, COL>, ROW> cameFrom;

    //Methods
    std::deque<pair> getPath(pair start, pair goal);
    void AStar(pair start, pair goal);
    void BFS(pair start, pair goal);
    void GBFS(pair start, pair goal);
    void pairToNode(pair var1, node &var2);

};

#endif // PATHPLANNER_H
