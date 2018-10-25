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
//const int key_left = 81;
//const int key_up = 82;
//const int key_down = 84;
//const int key_right = 83;
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
        //std::cout << std::setprecision(3) << "x: " << _gazeboWorld.getXPos() << " y: " << _gazeboWorld.getYPos() <<" a: "<< _gazeboWorld.getAngle() << std::endl;
        double pi = 3.14159;

        double end_x = 1;
        double end_y = 1;
        double start_x = _gazeboWorld.getXPos();
        double start_y = _gazeboWorld.getYPos();

        double ac_x = end_x - start_x;
        double ac_y = end_y - start_y;

        double angle_point = asin((ac_x)/sqrt((ac_x*ac_x)+(ac_y*ac_y)));

        double goal = _gazeboWorld.getAngle() - angle_point;

        if (goal < (-1*pi))
        {
            goal = -1*(goal + 2*abs(goal)-pi);
        }
        else if ((_gazeboWorld.getAngle() - angle_point) > (pi))
        {
            goal = -1*(goal - 2*(goal-pi));
        }


        std::cout << std::setprecision(3) << goal << std::endl;

        std::array<float,200> range_array=cvObj.getLidarRange();
        bool circle_bool=cvObj.getCircleBool();
        int offset=cvObj.getOffset();

        cvObj.seeCameraNew();
        cvObj.seeLidarNew();

        if(cvObj.getCameraLock() && cvObj.getLidarLock())
        {
            AI.fuzzyUpdate(range_array,circle_bool,offset);

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
