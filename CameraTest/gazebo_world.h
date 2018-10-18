#ifndef GAZEBO_WORLD_H
#define GAZEBO_WORLD_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

class gazebo_world
{
public:
    gazebo_world();
    gazebo_world(gazebo::transport::NodePtr &node);

    void startStat();
    void startPose();
    void generatePose(double speed,double dir);
    void worldReset();

    gazebo::transport::NodePtr getNode();



private:
    static void statCallback(ConstWorldStatisticsPtr &_msg);
    static void poseCallback(ConstPosesStampedPtr &_msg);



    gazebo::transport::NodePtr node;
    gazebo::transport::PublisherPtr movementPublisher;
    gazebo::transport::PublisherPtr worldPublisher;
    gazebo::transport::SubscriberPtr statSubscriber;
    gazebo::transport::SubscriberPtr poseSubscriber;

};

#endif // GAZEBO_WORLD_H
