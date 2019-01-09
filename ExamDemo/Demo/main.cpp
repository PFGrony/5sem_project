#include <mutex>
// Classes
#include "mapPlanning.h"
#include "QLearning.h"
#include "computerVision.h"
#include "gazeboWorld.h"
#include "fuzzyController.h"
#include "pathPlanner.h"

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
    cout << "Critical points:" << endl;
    mapPlanning planner("floor_plan.png");
    planner.calculateMap();

    // pathPlanner
    pathPlanner pathObj("floor_plan.png");

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

    ppPair start{0,0};
    ppPair goal{0,0};

    deque<ppPair> path;

    int QLpathCounter = 0;
    int stayCounter = 100;

    double distanceRobToPos = 0;

    double posX = 0;
    double posY = 0;

    cin.get();

    while(path.empty())
    {
        start.x = QL.getPoint(QLpathCounter).x;
        start.y = QL.getPoint(QLpathCounter).y;

        goal.x = QL.getPoint(QLpathCounter+1).x;
        goal.y = QL.getPoint(QLpathCounter+1).y;

        path = pathObj.AStarPlan(start,goal);
    }

    int pathLengths = path.size();
    cout << "Start " << start.x << ":" << start.y << " Goal " << goal.x << ":" << goal.y << endl;
    QLpathCounter++;
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

        if(stayCounter < 2 && pathLengths < 3)
        {

            start.x = QL.getPoint(QLpathCounter).x;
            start.y = QL.getPoint(QLpathCounter).y;

            goal.x = QL.getPoint(QLpathCounter+1).x;
            goal.y = QL.getPoint(QLpathCounter+1).y;
                deque<ppPair> newPath = pathObj.AStarPlan(start,goal);
                path.swap(newPath);
                pathLengths = path.size();
                cout << "Start " << start.x << ":" << start.y << " Goal " << goal.x << ":" << goal.y << endl;
                QLpathCounter++;
        }

        if (distanceRobToPos < 2 && pathLengths > 2)
        {
                pathLengths--;
                posX = (path.begin()->x-60)/1.44;
                posY = (40-path.begin()->y)/1.44;
                path.pop_front();
                stayCounter = 1000;
        }

        if (pathLengths < 3)
        {
            if (stayCounter > 1)
                stayCounter--;
        }

        // Robot pose in gazeboworld
        double robX = _gazeboWorld.getXPos();
        double robY = _gazeboWorld.getYPos();
        double robA = _gazeboWorld.getAngle();
        distanceRobToPos = sqrt(pow(posX-robX,2)+pow(posY-robY,2));
        //cout << distanceRobToPos << endl;
        //cout << robX << ":" << robY << endl;
        // Ball distance
        if (cvObj.getCircleBool())
        {
            std::pair<double,double> marblePos=cvObj.getMarblePos({robX,robY},robA);
            std::cout<<"Calculated marble position: ("<<(marblePos.first*1.44)+60<<","<<80-((marblePos.second*1.44)+40)<<")"<<std::endl;
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
