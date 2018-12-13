#ifndef COMPUTERVISION_H
#define COMPUTERVISION_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "opencv2/opencv.hpp"

static boost::mutex mutex;

class computerVision
{
public:
    explicit computerVision(gazebo::transport::NodePtr);

    //LIDAR
    bool getLidarLock();
    float* getLidarAngle();
    float* getLidarRange();

    //Camera
    std::pair<double, double> getMarblePos(std::pair<double, double> robPos,double robAngle);
    void seeCamera();
    void seeCameraV1();
    void seeCameraV2();

    bool getCameraLock();
    bool getCircleBool();
    int getOffset();
    float getRadius();

    //LIDAR
    void seeLidar();
    void seeLidarV1();

    void templateMatching();

private:
    //Gazebo
    void startCamera(gazebo::transport::NodePtr &node);
    void startLidar(gazebo::transport::NodePtr &node);
    static void cameraCallback(ConstImageStampedPtr &msg);
    static void lidarCallback(ConstLaserScanStampedPtr &msg);
    gazebo::transport::SubscriberPtr cameraSubscriber;
    gazebo::transport::SubscriberPtr lidarSubscriber;

    int offset=160;
    bool circle_bool=0;

    float ballRadius=0;

    cv::Mat templ;


};
#endif // COMPUTERVISION_H
