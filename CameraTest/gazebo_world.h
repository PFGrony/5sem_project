#ifndef GAZEBO_WORLD_H
#define GAZEBO_WORLD_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

class gazebo_world
{
public:
    gazebo_world();

    void startStat(gazebo::transport::NodePtr &temp);
    void startPose(gazebo::transport::NodePtr &temp);

private:
    static void statCallback(ConstWorldStatisticsPtr &_msg);
    static void poseCallback(ConstPosesStampedPtr &_msg);

};

#endif // GAZEBO_WORLD_H
