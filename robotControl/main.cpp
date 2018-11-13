#include <iostream>



//OpenCV
#include "opencv2/opencv.hpp"

//Classes

#include "pathPlanner.h"



int main()
{


	//Path planner
	pathPlanner plan;


	plan.wavefrontPlanner(pair{1,2}, pair{120-3,80-3});
	cv::imshow("map", plan.getMapWave());
	cv::waitKey();


	return 0;
}
