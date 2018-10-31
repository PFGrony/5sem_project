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

    //Generate Map
    generateMap mapObj;

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

        std::array<float,200> range_array=cvObj.getLidarRange();
        bool circle_bool=cvObj.getCircleBool();
        int offset=cvObj.getOffset();

        cvObj.seeCameraNew();
        cvObj.seeLidarNew();

        if(true && cvObj.getCameraLock() && cvObj.getLidarLock())
        {
            AI.fuzzyUpdate(range_array,circle_bool,offset);

            // Generate a pose
            _gazeboWorld.generatePose(AI.getSpeed(),AI.getSteer());
        }
        else
        {
            _gazeboWorld.generatePose(0,0);
        }

        //mapObj.calculateRobotPos(AI.getSpeed(),AI.getSteer());
        //mapObj.setRobPos();

        if(cvObj.getLidarLock())
        {
            mapObj.calculateObstaclePoints(range_array);
        }

                mapObj.insertPointsOnMap();

        cv::imshow("bla",mapObj.getMat());
        cv::imwrite("mappingMap.png",mapObj.getMat());

    }

    // Resets
    _gazeboWorld.generatePose(0,0);
    // Make sure to shut everything down.
    gazebo::client::shutdown();

    return 0;
}
