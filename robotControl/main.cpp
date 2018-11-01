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


    float speed = 0.0;
    float dir = 0.0;

    // maple (x,y)
    double mapleX = 1.0;
    double mapleY = 1.0;

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

        cvObj.seeCameraV2();
        cvObj.seeLidarNew();

        // Robot pose in gazeboworld
        double robX = _gazeboWorld.getXPos();
        double robY = _gazeboWorld.getYPos();
        double robA = _gazeboWorld.getAngle();

        //Mapping
        mutex.lock();
        mapObj.calculateRobotPos(AI.getSpeed(),AI.getSteer());

        std::cout << std::setprecision(3) << "X: " << (mapObj.getXPos() - robX) << " Y: " << (mapObj.getYPos() - robY) << " A: " << (mapObj.getAngle() - robA) << std::endl;

        mapObj.setRobPos(robX,robY,robA);
        mutex.unlock();


        mutex.lock();
        mapObj.insertPointsOnMap();
        mutex.unlock();

        if(cvObj.getLidarLock())
        {
            mutex.lock();
            mapObj.calculateObstaclePoints(lidarArray);
            mutex.unlock();
        }


        // Ball distance
        if (cvObj.getCircleBool())
        {
            double knownPixRadius = 29.0; // størrelse i pixels på maple i smallworld, når man står i starten
            double knownRadius = 0.5; // radius på maples
            double knownDistance = 5; // afstand fra robotens center (0,0) til maple center (5,0) i smallworld

            double focalLength = (knownPixRadius * knownDistance)/knownRadius;
            double distance = (knownRadius * focalLength)/cvObj.getRadius();

            double mapleAngle = -1* cvObj.getOffset() * (1.047/320) + robA; // FOV: 1.047 rad, pixel width 320

            mapleX = distance * std::cos(mapleAngle) + robX;
            mapleY = distance * std::sin(mapleAngle) + robY;

            //std::cout << mapleX << " : " << mapleY << std::endl;
        }

        if(true && cvObj.getCameraLock() && cvObj.getLidarLock())
        {
            AI.fuzzyUpdate(lidarArray,robX,robY,robA,mapleX,mapleY);
            // Generate a pose
            _gazeboWorld.generatePose(AI.getSpeed(),AI.getSteer());
        }
        else if(false)
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
