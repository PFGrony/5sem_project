//std classes
#include <iostream>

//Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

//OpenCV
#include "opencv2/opencv.hpp"
#include "fl/Headers.h"

//Classes
#include "computerVision.h"
#include "gazeboWorld.h"
#include "fuzzyController.h"


//Key constants
//    const int key_left = 81;
//    const int key_up = 82;
//    const int key_down = 84;
//    const int key_right = 83;
#define key_esc  27

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

    int circle_bool = 0;
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

      //Container for data from camera class
      std::array<float,200> something=cvObj.getLidarRange();

      //Camera data
      cv::Mat imageCopy=cvObj.getMatCamera();


      //detect circles
      int offset = (160);
      int rad = 0;


      if (! imageCopy.empty()) // find circles
      {
          cv::Mat gray;
          cv::cvtColor(imageCopy, gray, cv::COLOR_BGR2GRAY);
          cv::medianBlur(gray, gray, 5);

          std::vector<cv::Vec3f> circles;
          cv::HoughCircles(gray,circles, cv::HOUGH_GRADIENT,1,gray.rows,50,20,0,0);

          if (circles.size() > 0)
              circle_bool = 1;
          else
              circle_bool = 0;


          for( size_t i = 0; i < circles.size(); i++ )
          {
              cv::Vec3i c = circles[i];
              cv::Point center = cv::Point(c[0], c[1]);

              if (abs(int(c[0])-160)<abs(offset) && int(c[2]) > rad)
              {
                  offset = int(c[0])-160;
                  rad = int(c[2]);
                  //std::cout << "off: " << offset << ", rad: " << rad <<  std::endl;
              }

          }
      }

      AI.fuzzyUpdate(something,circle_bool,offset);

      _gazeboWorld.generatePose(AI.getSpeed(),AI.getSteer());

    }
    // Make sure to shut everything down.
    gazebo::client::shutdown();

    return 0;
}
