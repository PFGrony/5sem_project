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
    void wavefrontPlanner(int startX,int startY,int goalX,int goalY);
    void addAdj(std::deque<pair> &queueAdj);



private:
    cv::Mat smallMap=cv::imread("../robotControl/floor_plan.png",CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat map;
    std::vector<vertex> ballList;
    char charMap[80][120];
    char newMap[80][120];


};

#endif // PATHPLANNER_H
