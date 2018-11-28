#include <iostream>

//Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

//OpenCV
#include "opencv2/opencv.hpp"

//Classes
#include "computerVision.h"
#include "gazeboWorld.h"
#include "fuzzyController.h"
#include "generateMap.h"
#include "QLearning.h"

//Key constants
const int key_left = 81;
const int key_up = 82;
const int key_down = 84;
const int key_right = 83;
const int key_esc = 27;
const int key_enter = 10;

static boost::mutex mutexRB;

int main()
{   
    //Create a Gazebo World
    gazeboWorld _gazeboWorld;

	//Get Gazebo World pointer
	gazebo::transport::NodePtr node = _gazeboWorld.getNode();

	//resets Gazebo World
	_gazeboWorld.worldReset();


    //Camera Functions class
    computerVision cvObj;

    cvObj.startCamera(node);
    cvObj.startLidar(node);

    //resets Gazebo World
    _gazeboWorld.worldReset();

    //Run Q-Learning
    QLearning QL;
    QL.runQLearning();

    QL.calculateaiTable();

    int QLcounter = 0;

    // Start AI of doom
    fuzzyController AI;
    AI.fuzzyInit();


    //Path planner
    pathPlanner plan;

    // maple (x,y)
    double mapleX = -20.0;
    double mapleY = 0.0;


	// Loop
	while (true)
	{
		//Waits for 10ms in gazebo
		gazebo::common::Time::MSleep(10);

        //Get key input
        mutexRB.lock();
        int key = cv::waitKey(1);
        mutexRB.unlock();

        //Checks key input
        if (key == key_esc)
            break;
        else if(key == key_enter)
            QLcounter++;

		float* lidarArray = cvObj.getLidarRange();

        cvObj.seeCameraV2();
		cvObj.seeLidarNew();

		// Robot pose in gazeboworld
		double robX = _gazeboWorld.getXPos();
		double robY = _gazeboWorld.getYPos();
		double robA = _gazeboWorld.getAngle();

		// Template Matching
//        cvObj.templateMatching();


        //Mapping
        //double imgX = (map.cols/2)+robX*5.6;
        //double imgY = (map.rows/2)-robY*5.6;


        // Ball distance
        if (cvObj.getCircleBool())
        {
            double knownPixRadius = 29.0; // størrelse i pixels på maple i smallworld, når man står i starten
            double knownRadius = 0.5; // radius på maples
            double knownDistance = 5; // afstand fra robotens center (0,0) til maple center (5,0) i smallworld


        cv::imshow("map",plan.getMapWave());
	}

	// Resets
	_gazeboWorld.generatePose(0, 0);
	// Make sure to shut everything down.
	gazebo::client::shutdown();

            //std::cout << mapleX << " : " << mapleY << std::endl;
        }

        double x = (QL.getPoint(QLcounter).x-60)/1.5;
        double y = (40-QL.getPoint(QLcounter).y)/1.5;

        std::cout << x << ":" << y << std::endl;

        if(true && cvObj.getCameraLock() && cvObj.getLidarLock())
        {
            AI.fuzzyUpdate(lidarArray,robX,robY,robA,x,y);
            // Generate a pose
            _gazeboWorld.generatePose(AI.getSpeed(),AI.getSteer());
        }
        else if(true)
        {
            if ((key == key_up) && (speed <= 1.2f))
              speed += 0.05;
            else if ((key == key_down) && (speed >= -1.2f))
              speed -= 0.05;
            else if ((key == key_right) && (dir <= 0.4f))
              dir += 0.05;
            else if ((key == key_left) && (dir >= -0.4f))
              dir -= 0.05;
            else {
              // slow down
                    speed *= 0.99;
                    dir *= 0.99;
            }
            _gazeboWorld.generatePose(speed,dir);
        }
        else
        {
            _gazeboWorld.generatePose(0,0);
        }


//        if(doOnce==1)
//        {
//            plan.doBrushfire();
//            doOnce=0;
//        }

    }

    // Resets
    _gazeboWorld.generatePose(0,0);
    // Make sure to shut everything down.
    gazebo::client::shutdown();

    return 0;
}
