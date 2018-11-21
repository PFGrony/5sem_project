#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "opencv2/opencv.hpp"

#include <array>
#include <vector>
#include <deque>
#include <queue>
#include <cmath>


#define THRESHOLD 4.2

////Big Map
//#define ROW  80
//#define COL  120
//Small Map
#define ROW  15
#define COL  20

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

struct barVal
{
	double val = -1;
	double amount = 1;
	pair pairs = {0,0};
	bool seen = false;
};



class pathPlanner
{
public:


    pathPlanner();
    cv::Mat getMap();
    void doBrushfire();
    void voronoiDiagram();


    //Wavefront
	cv::Mat getMapWave()
	{
		return mapWave;
	}
	cv::Mat getMapCopy()
	{
		return mapCopy;
	}
    void wavefrontRoute(pair start, pair goal);
    void wavefrontPlanner(pair start, pair goal);
    std::deque<pair> getWavefrontRoute();
    void drawWavefrontRoute(pair start,pair goal);
    void drawWavefrontBrushfire(pair start,pair goal);

	//AGP
	void AGP();
	std::deque<pair> getCriticalPoints();
	cv::Mat getPoints()
	{
		return points;
	}


    //A Star
	void pairToNode(pair var1, node &var2);
	void AStar(pair start, pair goal);
	void BFS(pair start, pair goal);
	void GBFS(pair start, pair goal);
	std::deque<pair> getPath(pair start, pair goal);
	void drawPath(pair start, pair goal);
	void drawAStar(pair start, pair goal);
	void printCameFrom();





private:
	//Init
	cv::Mat map = cv::imread("../robotControl/floor_plan.png", CV_LOAD_IMAGE_ANYCOLOR);
	cv::Mat smallMap; //cv::imread(/*"../robotControl/*/"floor_plan.png",CV_LOAD_IMAGE_GRAYSCALE);


    //Wavefront
    int intMap[ROW][COL];
    int cameFromMap[ROW][COL];
	cv::Mat mapWave = map.clone();
    std::deque<pair> routelist;

	//BFS,GBFS, AStar
	std::array<std::array<pair, COL>, ROW> cameFrom;
	cv::Mat mapCopy;

	//AGP
	std::deque<pair> criticalPoints;
	cv::Mat points;

	//MISC
	std::vector<node> lister;
};

#endif // PATHPLANNER_H
