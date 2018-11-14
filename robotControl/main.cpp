#include <iostream>



//OpenCV
#include "opencv2/opencv.hpp"

//Classes

#include "pathPlanner.h"



int main()
{


	//Path planner
	pathPlanner plan;


    plan.wavefrontPlanner(pair{1,2}, pair{17,14});
    plan.wavefrontRoute(pair{1,2}, pair{17,14});
    std::deque<pair> list=plan.getWavefrontRoute();

    plan.drawWavefrontRoute(pair{1,2}, pair{17,14});
	cv::imshow("map", plan.getMapWave());
	cv::waitKey();


	return 0;
}
