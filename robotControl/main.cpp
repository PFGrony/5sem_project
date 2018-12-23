///Classes
#include "gazeboWorld.h"

#include "mapPlanning.h"
#include "QLearning.h"

#include "computerVision.h"
//#include "pathPlanner.h"
#include "fuzzyController.h"


//Key constants
const int key_left = 81;
const int key_up = 82;
const int key_down = 84;
const int key_right = 83;
const int key_esc = 27;
const int key_enter = 10;

const bool automated = false;

int main()
{
    //Creata Gazebo World
    gazeboWorld _gazeboWorld;
    //Get Gazebo World pointer
    gazebo::transport::NodePtr node = _gazeboWorld.getNode();
    //resets Gazebo World
    _gazeboWorld.worldReset();
    //Camera Functions class
    computerVision cvObj(node);

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
        mutexCV.lock();
        int key = cv::waitKey(1);
        mutexCV.unlock();

        //Checks key input
        if (key == key_esc)
            break;

        float* lidarArray = cvObj.getLidarRange();

        cvObj.seeCameraV2();
        cvObj.seeLidarV1();

        // Robot pose in gazeboworld
        std::pair<double,double> robPos(_gazeboWorld.getXPos(),_gazeboWorld.getYPos());
        double robAngle = _gazeboWorld.getAngle();

        // Ball distance
        std::pair<double, double> marblePos=cvObj.getMarblePos(robPos, robAngle);

        if (automated && cvObj.getCameraLock() && cvObj.getLidarLock())
        {
            AI.fuzzyUpdate(lidarArray, robPos.first, robPos.second, robAngle, marblePos.first, marblePos.second);
            // Generate a pose
            _gazeboWorld.generatePose(AI.getSpeed(), AI.getSteer());
        }
        else if (!automated)
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
            _gazeboWorld.generatePose(speed, dir);
        }
        else
        {
            _gazeboWorld.generatePose(0, 0);
        }
    }

    // Resets
    _gazeboWorld.generatePose(0, 0);
    // Make sure to shut everything down.
    gazebo::client::shutdown();

    return 0;
}
