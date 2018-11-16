#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "opencv2/opencv.hpp"

#include <array>
#include <vector>
#include <deque>

struct node
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


const int ROW = 80;
const int COL = 120;

class pathPlanner
{
public:


    pathPlanner();
    cv::Mat getMap();
    void doBrushfire();


    //Wavefront
	cv::Mat getMapWave()
	{
		return mapWave;
	}
    void wavefrontRoute(pair start, pair goal);
    void wavefrontPlanner(pair start, pair goal);
    std::deque<pair> getWavefrontRoute();
    void drawWavefrontRoute(pair start,pair goal);
    void drawWavefrontBrushfire(pair start,pair goal);

    //A Star
    std::vector<pair> AStar(pair start, pair goal);
    void voronoiDiagram();
	void drawAStar(pair start, pair goal);

private:
    cv::Mat smallMap=cv::imread("../robotControl/floor_plan.png",CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat map= cv::imread("../robotControl/floor_plan.png", CV_LOAD_IMAGE_ANYCOLOR);

	std::vector<pair> lister;


    //Wavefront
    int charMap[ROW][COL];
    int newMap[ROW][COL];
	cv::Mat mapWave = map.clone();
    std::deque<pair> routelist;


};

#endif // PATHPLANNER_H
