#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>

#include "fl/Headers.h"

#include "fuzzycontroller.h"


static boost::mutex mutex;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  /*for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {

      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6)
                << _msg->pose(i).position().y() << std::setw(6)
                << _msg->pose(i).position().z() << std::setw(6)
                << _msg->pose(i).orientation().w() << std::setw(6)
                << _msg->pose(i).orientation().x() << std::setw(6)
                << _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;
    }
  }*/
}

int offset = (160);
int circle_bool = 0;


void cameraCallback(ConstImageStampedPtr &msg)
{
    std::size_t width = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    //int(height) 240
    //int(width) 320

    offset = (160);
    int rad = 0;

    if (true) // find circles
    {
        cv::Mat gray;
        cv::cvtColor(im, gray, cv::COLOR_BGR2GRAY);
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

            // circle center
            //cv::circle( im, center, 1, cv::Scalar(255,0,0), 3, cv::LINE_AA);
            // circle outline
            //int radius = c[2];
            //cv::circle( im, center, radius, cv::Scalar(255,0,0), 3, cv::LINE_AA);
        }
    }

    im = im.clone();
    cv::cvtColor(im, im, CV_BGR2RGB);

    mutex.lock();
    cv::imshow("camera", im);
    mutex.unlock();

}

float range_array[200] = { };

void lidarCallback(ConstLaserScanStampedPtr &msg) {

  //  std::cout << ">> " << msg->DebugString() << std::endl;
  float angle_min = float(msg->scan().angle_min());
  //  double angle_max = msg->scan().angle_max();
  float angle_increment = float(msg->scan().angle_step());

  //std::cout << "Angle_min: " << angle_min << " angle_inc: " << angle_increment << std::endl;

  float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());

  //std::cout << "min: " << range_min << std::endl;
  //std::cout << "max: " << range_max << std::endl;

  int sec = msg->time().sec();
  int nsec = msg->time().nsec();

  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

  assert(nranges == nintensities);

  int width = 400;
  int height = 400;
  float px_per_m = 200 / range_max;

  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);
  for (int i = 0; i < nranges; i++)
  {
      range_array[i]=std::min(float(msg->scan().ranges(i)), range_max);
      float angle = angle_min + i * angle_increment;
      float range = std::min(float(msg->scan().ranges(i)), range_max);
      cv::Scalar color;
      if (i<60 || i>140)
          color = cv::Scalar(0,255,255);
      if (i>59 && i<141)
          color = cv::Scalar(0,255,0);
      if (i>89 && i<111)
          color = cv::Scalar(0,0,255);
    //    double intensity = msg->scan().intensities(i);
      cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                        200.5f - range_min * px_per_m * std::sin(angle));
      cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                      200.5f - range * px_per_m * std::sin(angle));
      cv::line(im, startpt * 16, endpt * 16, color, 1,
             cv::LINE_AA, 4);

    //    std::cout << angle << " " << range << " " << intensity << std::endl;
  }
  cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
  cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

  mutex.lock();
  cv::imshow("lidar", im);
  mutex.unlock();
}



int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  gazebo::transport::SubscriberPtr statSubscriber =
      node->Subscribe("~/world_stats", statCallback);

  gazebo::transport::SubscriberPtr poseSubscriber =
      node->Subscribe("~/pose/info", poseCallback);

  gazebo::transport::SubscriberPtr cameraSubscriber =
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

  gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

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

  // Start AI of doom
  fuzzyController AI;
  AI.fuzzyInit();

  // Loop
  while (true)
  {
    gazebo::common::Time::MSleep(10);

    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();

    if (key == 27)
        break;


    AI.fuzzyUpdate(range_array,circle_bool,offset);

    //std::cout << "dir: "<< AI.getSteer() << " speed: "<< AI.getSpeed() << std::endl;

    // Generate a pose
    ignition::math::Pose3d pose(double(AI.getSpeed()), 0, 0, 0, 0, double(AI.getSteer()));

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
