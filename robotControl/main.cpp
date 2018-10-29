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

        cvObj.seeCameraNew();
        cvObj.seeLidarNew();

        // Robot pose in gazeboworld
        double robX = _gazeboWorld.getXPos();
        double robY = _gazeboWorld.getYPos();
        double robA = _gazeboWorld.getAngle();

        // Robot distination in gazeboworld
        double distX = 20;
        double distY = -10;

        if(cvObj.getCameraLock() && cvObj.getLidarLock())
        {
            AI.fuzzyUpdate(lidarArray,robX,robY,robA,distX,distY);
            // Generate a pose
            _gazeboWorld.generatePose(AI.getSpeed(),AI.getSteer());
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
