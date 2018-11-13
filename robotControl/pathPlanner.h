#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "opencv2/opencv.hpp"

#include <array>
#include <vector>
#include <deque>

struct vertex
{
    int currentNode;
    int previousNode;
    int x;
    int y;
    int g;//distance;
    int h;//heuristicDistance;
    int f;
};
struct pair
{
    int x;
    int y;
};

class pathPlanner
{
public:


    pathPlanner();
    cv::Mat getMap();
    void doBrushfire();
    void AStar(int goalX,int goalY);
    void addVertex(int x, int y);
    void wavefrontPlanner(pair start, pair goal);
    void addAdj(std::deque<pair> &queueAdj);
	cv::Mat getMapWave()
	{
		return mapWave;
	}
	cv::Mat mapWaveArry()
	{
		return cv::Mat(80,120,CV_16U,&newMap);
	}


private:
    cv::Mat smallMap=cv::imread("floor_plan.png",CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat map;
    std::vector<vertex> ballList;
    int charMap[80][120];
    int newMap[80][120];
	cv::Mat mapWave = cv::imread("floor_plan.png", CV_LOAD_IMAGE_ANYCOLOR);

};

#endif // PATHPLANNER_H
