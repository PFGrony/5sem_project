#include "gazeboWorld.h"

static double xPos;
static double yPos;
static double angle;

gazeboWorld::gazeboWorld()
{
    angle = 0;
    xPos = 0;
    yPos = 0;
    // Load gazebo
    gazebo::client::setup();

    // Create our node for communication
    gazebo::transport::NodePtr nodeNew(new gazebo::transport::Node());
    node=nodeNew;
    node->Init();

    // Publish to the robot vel_cmd topic
    movementPublisher = node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    //starts Gazebo statistics and posing
    this->startStat();
    this->startPose();

}

gazebo::transport::NodePtr gazeboWorld::getNode()
{
    return node;
}

void gazeboWorld::statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void gazeboWorld::poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++)
  {
    if (_msg->pose(i).name() == "pioneer2dx")
    {
        // copy paste fra wiki
        double siny_cosp = +2.0 * (_msg->pose(i).orientation().w() * _msg->pose(i).orientation().z() + _msg->pose(i).orientation().x() * _msg->pose(i).orientation().y());
        double cosy_cosp = +1.0 - 2.0 * (_msg->pose(i).orientation().y() * _msg->pose(i).orientation().y() + _msg->pose(i).orientation().z() * _msg->pose(i).orientation().z());
        angle = atan2(siny_cosp, cosy_cosp);

        xPos = _msg->pose(i).position().x();
        yPos = _msg->pose(i).position().y();
    }
  }
}

void gazeboWorld::startStat()
{
    statSubscriber = node->Subscribe("~/world_stats", statCallback);
}

void gazeboWorld::startPose()
{
    poseSubscriber = node->Subscribe("~/pose/info", poseCallback);
}

void gazeboWorld::generatePose(double speed,double dir)
{
    // Generate a pose
    ignition::math::Pose3d pose(speed, 0, 0, 0, 0, dir);

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
}

void gazeboWorld::worldReset()
{
    // Publish a reset of the world
    worldPublisher = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);
}

double gazeboWorld::getXPos()
{
    return xPos;
}

double gazeboWorld::getYPos()
{
    return yPos;
}

double gazeboWorld::getAngle()
{
    return angle;
}
