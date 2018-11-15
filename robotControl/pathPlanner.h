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
    void AStar(pair start, pair goal);
    void wavefrontPlanner(pair start, pair goal);


    //Wavefront
	cv::Mat getMapWave()
	{
		return mapWave;
	}
    void wavefrontRoute(pair start, pair goal);
    std::deque<pair> getWavefrontRoute();
    void drawWavefrontRoute(pair start,pair goal);
    void drawWavefrontBrushfire(pair start,pair goal);

    //A Star
    void voronoiDiagram();


private:
    cv::Mat smallMap=cv::imread("../robotControl/floor_plan.png",CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat map;


    //Wavefront
    int charMap[80][120];
    int newMap[80][120];
    cv::Mat mapWave = cv::imread("../robotControl/floor_plan.png", CV_LOAD_IMAGE_ANYCOLOR);
    std::deque<pair> routelist;


};

#endif // PATHPLANNER_H
