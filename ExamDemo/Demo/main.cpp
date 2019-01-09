#include <mutex>
// Classes
#include "mapPlanning.h"
#include "QLearning.h"
#include "computerVision.h"
#include "gazeboWorld.h"
#include "fuzzyController.h"

using namespace std;

//Key constants
const int key_left = 81;
const int key_up = 82;
const int key_down = 84;
const int key_right = 83;
const int key_esc = 27;


int main()
{
    // mapPlanning
    mapPlanning planner("floor_plan.png");
    planner.calculateMap();

    // QLearning
    QLearning QL;
    QL.importMap(planner.getPathVec());
    QL.runQLearning();
    QL.printBestActions();

    // gazeboWorld
    gazeboWorld _gazeboWorld;
    gazebo::transport::NodePtr node = _gazeboWorld.getNode();
    _gazeboWorld.worldReset();

    // computerVision
    computerVision cvObj(node);

    // fuzzyController
    fuzzyController AI;
    AI.fuzzyInit();

    //Mutex
    std::mutex demo_mutex;

    //Control variables
    float speed = 0.0;
    float dir = 0.0;

    while (true)
    {
        //Waits for 10ms in gazebo
        gazebo::common::Time::MSleep(10);

        //Get key input
        demo_mutex.lock();
        int key = cv::waitKey(1);
        demo_mutex.unlock();

        //Checks key input
        if (key == key_esc)
            break;

        float* lidarArray = cvObj.getLidarRange();

        cvObj.seeCameraV2();
        cvObj.seeLidarV1();

        // Robot pose in gazeboworld
        double robX = _gazeboWorld.getXPos();
        double robY = _gazeboWorld.getYPos();
        double robA = _gazeboWorld.getAngle();

        double posX = (QL.getPoint(3).x-60)*1.44;
        double posY = (40-QL.getPoint(3).y)*1.44;

        cout << posX << ":" << posY << endl;

        double distanceRobToPos = sqrt(pow(posX-robX,2)+pow(posY-robY,2));


        // Ball distance
        if (cvObj.getCircleBool())
        {
            std::pair<double,double> marblePos=cvObj.getMarblePos({robX,robY},robA);
            //std::cout<<"Calculated: ("<<marblePos.first<<","<<marblePos.second<<")"<<std::endl;
            //std::cout<<"Own position: ("<<robX<<","<<robY<<")"<<std::endl;
            //std::cout<<"Angle: "<<robA<<std::endl;

            //fs1 << marblePos.first<<","<<marblePos.second<< std::endl;
            //fs2 << robX<<","<<robY<< std::endl;
            //std::cout << mapleX << " : " << mapleY << std::endl;
        }

        if (true && cvObj.getCameraLock() && cvObj.getLidarLock())
        {
            AI.fuzzyUpdate(lidarArray, robX, robY, robA, posX, posY);
            // Generate a pose
            _gazeboWorld.generatePose(AI.getSpeed(), AI.getSteer());
        }
        else if (false)
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
