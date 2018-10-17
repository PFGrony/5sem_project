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

    gazebo::transport::SubscriberPtr startCamera(gazebo::transport::NodePtr &temp);
    gazebo::transport::SubscriberPtr startLidar(gazebo::transport::NodePtr &temp);


    bool getLock();

    cv::Mat getMatCamera();
    std::array<float,200> getLidarAngle();
    std::array<float,200> getLidarRange();

private:
    static void cameraCallback(ConstImageStampedPtr &msg);
    static void lidarCallback(ConstLaserScanStampedPtr &msg);

};
#endif // CAMERA_H
