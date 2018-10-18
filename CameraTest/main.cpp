#include <iostream>

//Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

//OpenCV
#include "opencv2/opencv.hpp"

//Classes
#include "camera.h"
#include "gazebo_world.h"


int main()
{
    //Creata Gazebo World
    gazebo_world gazeboWorld;
    //Get Gazebo World pointer
    gazebo::transport::NodePtr node= gazeboWorld.getNode();
    //starts Gazebo statistics and posing
    gazeboWorld.startStat();
    gazeboWorld.startPose();



    //Camera Functions class
    camera camera_temp;

    camera_temp.startCamera(node);
    camera_temp.startLidar(node);


    //resets Gazebo World
    gazeboWorld.worldReset();

    const int key_left = 81;
    const int key_up = 82;
    const int key_down = 84;
    const int key_right = 83;
    const int key_esc = 27;

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

      //Container for data from camera class
      std::array<float,200> something=camera_temp.getLidarRange();

      //Prints Data from camera Class
      if(camera_temp.getLock()==1)
      {
          std::cout<<"Copy: [";
          for(int i=0;i<5;i++)
          {
              std::cout<<something[98+i];
              if(i!=4)
              {
                  std::cout<<", ";
              }
          }
          std::cout<<"]"<<std::endl;
      }


        gazeboWorld.generatePose(speed,dir);

    }
    // Make sure to shut everything down.
    gazebo::client::shutdown();

    return 0;
}
