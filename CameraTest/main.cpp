#include <iostream>
#include "camera.h"
#include "gazebo_world.h"



#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "opencv2/opencv.hpp"

//void statCallback(ConstWorldStatisticsPtr &_msg) {
//  (void)_msg;
//  // Dump the message contents to stdout.
//  //  std::cout << _msg->DebugString();
//  //  std::cout << std::flush;
//}

//void poseCallback(ConstPosesStampedPtr &_msg) {
//  // Dump the message contents to stdout.
//  //  std::cout << _msg->DebugString();

//  for (int i = 0; i < _msg->pose_size(); i++) {
//    if (_msg->pose(i).name() == "pioneer2dx") {

//      /*std::cout << std::setprecision(2) << std::fixed << std::setw(6)
//                << _msg->pose(i).position().x() << std::setw(6)
//                << _msg->pose(i).position().y() << std::setw(6)
//                << _msg->pose(i).position().z() << std::setw(6)
//                << _msg->pose(i).orientation().w() << std::setw(6)
//                << _msg->pose(i).orientation().x() << std::setw(6)
//                << _msg->pose(i).orientation().y() << std::setw(6)
//                << _msg->pose(i).orientation().z() << std::endl;*/
//    }
//  }
//}

int main()
{
    // Load gazebo
    gazebo::client::setup();

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo_world gazebo_temp;
    gazebo::transport::SubscriberPtr statSubscriber = gazebo_temp.startStat(node);
//        node->Subscribe("~/world_stats", statCallback);

    gazebo::transport::SubscriberPtr poseSubscriber = gazebo_temp.startPose(node);
//        node->Subscribe("~/pose/info", poseCallback);

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
    while (true) {
      gazebo::common::Time::MSleep(10);

      mutex.lock();
      int key = cv::waitKey(1);
      mutex.unlock();

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


      // Generate a pose
      ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

      // Convert to a pose message
      gazebo::msgs::Pose msg;
      gazebo::msgs::Set(&msg, pose);
      movementPublisher->Publish(msg);
    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();

//    Mat file=Mat::zeros(200,200,CV_8UC1)*255;
//    namedWindow("1",WINDOW_AUTOSIZE);
//    imshow("1",file);
//    cout << "Hello World!" << endl<<endl;
//    waitKey(0);

    return 0;
}
