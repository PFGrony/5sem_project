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

//Key constants
const int key_left = 81;
const int key_up = 82;
const int key_down = 84;
const int key_right = 83;
const int key_esc = 27;

int main()
{
    //Creata Gazebo World
    gazeboWorld _gazeboWorld;

    //Get Gazebo World pointer
    gazebo::transport::NodePtr node= _gazeboWorld.getNode();

    //Camera Functions class
    computerVision cvObj;

    cvObj.startCamera(node);
    cvObj.startLidar(node);

    //resets Gazebo World
    _gazeboWorld.worldReset();

    // Start AI of doom
    fuzzyController AI;
    AI.fuzzyInit();


    float speed = 0.0;
    float dir = 0.0;

    // Loop
    while (true)
    {
        //Waits for 10ms in gazebo
        gazebo::common::Time::MSleep(10);

        //Get key input
        mutex.lock();
        int key = cv::waitKey(1);
        mutex.unlock();

        //Checks key input
        if (key == key_esc)
            break;

        float* lidarArray = cvObj.getLidarRange();

//        cvObj.seeCameraNew();
        cvObj.seeCameraV2();
        cvObj.seeLidarNew();

        // Robot pose in gazeboworld
        double robX = _gazeboWorld.getXPos();
        double robY = _gazeboWorld.getYPos();
        double robA = _gazeboWorld.getAngle();

        // Robot distination in gazeboworld
        double distX = 20;
        double distY = -10;

        if(false && cvObj.getCameraLock() && cvObj.getLidarLock())
        {
            AI.fuzzyUpdate(lidarArray,robX,robY,robA,distX,distY);
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
    }

    // Resets
    _gazeboWorld.generatePose(0,0);
    // Make sure to shut everything down.
    gazebo::client::shutdown();

    return 0;
}
