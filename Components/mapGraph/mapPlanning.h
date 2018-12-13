#include "opencv2/opencv.hpp"
#include <queue>
#include <string>

#pragma once

#define THRESHOLD 4.3

struct coordinate
{
	int x;
	int y;
};

struct paths
{
	coordinate start;
	coordinate end;
	int cost;
    std::deque<coordinate> path;
};

//struct barVal
//{
//	double val = -1;
//	double amount = 1;
//	coordinate pairs = { 0,0 };
//	bool seen = false;
//};

class mapPlanning
{
public:
    explicit mapPlanning(std::string path);
	~mapPlanning();

    void setImgPath(std::string path);
	void showMap();
	void calculateMap();

	int getCrits();
	int getPathsCount();
    std::vector<paths> getPathVec();

private:
	void findCriticalPoints();
	int critPoints;

	void planMap();
    void planPath(std::vector<coordinate> conVec);

	int **intMap;

    std::vector<coordinate> criticalPoints;
    std::vector<coordinate> connectedPoints;

    std::vector<paths> pathVec;

	cv::Mat map;
	cv::Mat grayMap;
	cv::Mat mapWithCrits;
	cv::Mat mapWithPaths;
	cv::Mat points;
};

