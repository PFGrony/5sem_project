#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>


//OpenCV
#include "opencv2/opencv.hpp"

//Classes

#include "pathPlanner.h"



int main()
{

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

	//Path planner
	pathPlanner plan;

    pair start=pair{1,2};
    pair goal=pair{60,75};
    plan.wavefrontPlanner(start, goal);
    plan.wavefrontRoute(start,goal);
    std::deque<pair> list=plan.getWavefrontRoute();

//    plan.drawWavefrontRoute(start,goal);
//    cv::imshow("map", plan.getMapWave());
//    cv::waitKey();

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

     std::cout << "It took me " << time_span.count() << " seconds.";
     std::cout << std::endl;

	return 0;
}
