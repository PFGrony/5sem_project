///Classes
#include "computerVision.h"
#include "gazeboWorld.h"
#include "fuzzyController.h"
#include "pathPlanner.h"
#include "mapPlanning.h"


//Key constants
const int key_left = 81;
const int key_up = 82;
const int key_down = 84;
const int key_right = 83;
const int key_esc = 27;


const std::string imagePath="../robotControl/floor_plan.png";

int main()
{

    //Creata Gazebo World
    gazeboWorld _gazeboWorld;

    //Get Gazebo World pointer
    gazebo::transport::NodePtr node = _gazeboWorld.getNode();

    //resets Gazebo World
    _gazeboWorld.worldReset();

    //Camera Functions class
    computerVision cvObj;
    cvObj.startCamera(node);
    cvObj.startLidar(node);

    //Start AI of doom
    fuzzyController AI;
    AI.fuzzyInit();

    //Start Map Graph Creation
    mapPlanning planner(imagePath);
    planner.calculateMap();
    planner.showMap();
    cv::waitKey();

    float speed = 0.0;
    float dir = 0.0;

    // marble (x,y)
    double marbleX = 1.0;
    double marbleY = 1.0;

    std::fstream fs1, fs2;
    fs1.open ("marblePos1.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    fs2.open ("robotPos1.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    int q=0;
    
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
        cvObj.seeLidarV1();

        // Robot pose in gazeboworld
        double robX = _gazeboWorld.getXPos();
        double robY = _gazeboWorld.getYPos();
        double robA = _gazeboWorld.getAngle();

        // Template Matching
//        cvObj.templateMatching();

        // std::cout << std::setprecision(3) << "X: " << (mapObj.getXPos() - robX) << " Y: " << (mapObj.getYPos() - robY) << " A: " << (mapObj.getAngle() - robA) << std::endl;


        // Ball distance
        if (cvObj.getCircleBool())
        {
            double knownPixRadius = 29.0; // størrelse i pixels på marble i smallworld, når man står i starten
            double knownRadius = 0.5; // radius på marbles
            double knownDistance = 5; // afstand fra robotens center (0,0) til marble center (5,0) i smallworld

            double focalLength = (knownPixRadius * knownDistance) / knownRadius;
            double distance = (knownRadius * focalLength) / cvObj.getRadius();

            double marbleAngle = -1 * cvObj.getOffset() * (1.047 / 320) + robA; // FOV: 1.047 rad, pixel width 320

            marbleX = distance * std::cos(marbleAngle) + robX;
            marbleY = distance * std::sin(marbleAngle) + robY;

//            std::cout<<q++%100<<std::endl;
//            std::cout<<"Calculated: ("<<marbleX<<","<<marbleY<<")"<<std::endl;
            std::cout<<"Own position: ("<<robX<<","<<robY<<")"<<std::endl;
            std::cout<<"Angle: "<<robA<<std::endl;

//            fs1 << marbleX<<","<<marbleY<< std::endl;
//            fs2 << robX<<","<<robY<< std::endl;
        }



        if (false && cvObj.getCameraLock() && cvObj.getLidarLock())
        {
            AI.fuzzyUpdate(lidarArray, robX, robY, robA, marbleX, marbleY);
            // Generate a pose
            _gazeboWorld.generatePose(AI.getSpeed(), AI.getSteer());
        }
        else if (true)
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
    fs1.close();
    fs2.close();

    // Resets
    _gazeboWorld.generatePose(0, 0);
    // Make sure to shut everything down.
    gazebo::client::shutdown();

	return 0;
}
