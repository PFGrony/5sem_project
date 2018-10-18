#ifndef CAMERA_H
#define CAMERA_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "opencv2/opencv.hpp"

static boost::mutex mutex;

class camera
{
public:
    camera();

    bool getLock();

    cv::Mat getMatCamera();
    std::array<float,200> getLidarAngle();
    std::array<float,200> getLidarRange();


    void startCamera(gazebo::transport::NodePtr &node);
    void startLidar(gazebo::transport::NodePtr &node);


private:
    static void cameraCallback(ConstImageStampedPtr &msg);
    static void lidarCallback(ConstLaserScanStampedPtr &msg);


    gazebo::transport::SubscriberPtr cameraSubscriber;
    gazebo::transport::SubscriberPtr lidarSubscriber;

};
#endif // CAMERA_H
