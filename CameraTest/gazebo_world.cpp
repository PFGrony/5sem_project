#include "gazebo_world.h"

gazebo_world::gazebo_world()
{
}

void gazebo_world::statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void gazebo_world::poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {

      /*std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6)
                << _msg->pose(i).position().y() << std::setw(6)
                << _msg->pose(i).position().z() << std::setw(6)
                << _msg->pose(i).orientation().w() << std::setw(6)
                << _msg->pose(i).orientation().x() << std::setw(6)
                << _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;*/
    }
  }
}


gazebo::transport::SubscriberPtr gazebo_world::startPose(gazebo::transport::NodePtr &temp)
{
    return temp->Subscribe("~/pose/info", poseCallback);
}

gazebo::transport::SubscriberPtr gazebo_world::startStat(gazebo::transport::NodePtr &temp)
{
    return temp->Subscribe("~/world_stats", statCallback);
}
