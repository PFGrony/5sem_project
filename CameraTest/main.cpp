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
    // Load gazebo
    gazebo::client::setup();

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo_world gazeboWorld;
    gazeboWorld.startStat(node);
    gazeboWorld.startPose(node);

    //Camera Functions class
    camera camera_temp;
    gazebo::transport::SubscriberPtr cameraSubscriber = camera_temp.startCamera(node);
    gazebo::transport::SubscriberPtr lidarSubscriber = camera_temp.startLidar(node);



    // Publish to the robot vel_cmd topic
    gazebo::transport::PublisherPtr movementPublisher =
            node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    // Publish a reset of the world
    gazebo::transport::PublisherPtr worldPublisher =
        node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);

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
      //Waits for 10ms in gazebo and opencv
      gazebo::common::Time::MSleep(10);

      mutex.lock();
      int key = cv::waitKey(1);
      mutex.unlock();

      //Key input
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



      // Generate a pose
      ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

      // Convert to a pose message
      gazebo::msgs::Pose msg;
      gazebo::msgs::Set(&msg, pose);
      movementPublisher->Publish(msg);

    }
    // Make sure to shut everything down.
    gazebo::client::shutdown();

    return 0;
}
