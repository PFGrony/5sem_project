#include "gazebo_world.h"

gazebo_world::gazebo_world()
{
    // Load gazebo
    gazebo::client::setup();

    // Create our node for communication
    gazebo::transport::NodePtr nodeNew(new gazebo::transport::Node());
    node=nodeNew;
    node->Init();

    // Publish to the robot vel_cmd topic
    movementPublisher = node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

}

gazebo::transport::NodePtr gazebo_world::getNode()
{
    return node;
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

void gazebo_world::startStat()
{
    statSubscriber = node->Subscribe("~/world_stats", statCallback);
}

void gazebo_world::startPose()
{
    poseSubscriber = node->Subscribe("~/pose/info", poseCallback);
}

void gazebo_world::generatePose(double speed,double dir)
{
    // Generate a pose
    ignition::math::Pose3d pose(speed, 0, 0, 0, 0, dir);

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
}

void gazebo_world::worldReset()
{
    // Publish a reset of the world
    worldPublisher = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);
}
