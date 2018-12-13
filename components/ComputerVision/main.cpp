///Classes
#include "computerVision.h"
#include "gazeboWorld.h"

//Key constants
const int key_left = 81;
const int key_up = 82;
const int key_down = 84;
const int key_right = 83;
const int key_esc = 27;

const std::string imagePath="../../maps/floor_plan.png";

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

    //Control variables
    float speed = 0.0;
    float dir = 0.0;

    //Data
    std::fstream fs1, fs2;
    fs1.open ("../ComputerVision/marblePos1.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    fs2.open ("../ComputerVision/robotPos1.txt", std::fstream::in | std::fstream::out | std::fstream::app);
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

        cvObj.seeCameraV2();

        // Robot pose in gazeboworld
        double robX = _gazeboWorld.getXPos();
        double robY = _gazeboWorld.getYPos();
        double robA = _gazeboWorld.getAngle();


        // Ball distance
        if (cvObj.getCircleBool())
        {

            std::pair<double,double> marblePos=cvObj.getMarblePos({robX,robY},robA);
            std::cout<<q++%100<<std::endl;
            std::cout<<"Calculated: ("<<marblePos.first<<","<<marblePos.second<<")"<<std::endl;
            std::cout<<"Own position: ("<<robX<<","<<robY<<")"<<std::endl;
            std::cout<<"Angle: "<<robA<<std::endl;

            fs1 << marblePos.first<<","<<marblePos.second<< std::endl;
            fs2 << robX<<","<<robY<< std::endl;
        }



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
    fs1.close();
    fs2.close();

    // Resets
    _gazeboWorld.generatePose(0, 0);
    // Make sure to shut everything down.
    gazebo::client::shutdown();

    return 0;
}
