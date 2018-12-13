#include "opencv2/opencv.hpp"
#include <queue>
#include <string>

#pragma once

using namespace std;

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
	deque<coordinate> path;
};

struct barVal
{
	double val = -1;
	double amount = 1;
	coordinate pairs = { 0,0 };
	bool seen = false;
};

class mapPlanning
{
public:
	mapPlanning();
	mapPlanning(string path);
	~mapPlanning();

	void setImgPath(string path);
	void showMap();
	void calculateMap();

	int getCrits();
	int getPathsCount();
	vector<paths> getPathVec();

private:
	void findCriticalPoints();
	int critPoints;

	void planMap();
	void planPath(vector<coordinate> conVec);

	int **intMap;

	vector<coordinate> criticalPoints;
	vector<coordinate> connectedPoints;

	vector<paths> pathVec;

	cv::Mat map;
	cv::Mat grayMap;
	cv::Mat mapWithCrits;
	cv::Mat mapWithPaths;
	cv::Mat points;
};

