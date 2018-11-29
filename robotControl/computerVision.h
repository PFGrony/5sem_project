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
    computerVision();

    bool getLidarLock();
    bool getCameraLock();

    cv::Mat getMatCamera();
    float* getLidarAngle();
    float* getLidarRange();
    bool getCircleBool();
    int getOffset();
    float getRadius();


    void startCamera(gazebo::transport::NodePtr &node);
    void startLidar(gazebo::transport::NodePtr &node);

    void seeLidar();
    void seeLidarV1();

    void seeCamera();
    void seeCameraV1();
    void seeCameraV2();

    void templateMatching();

private:
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
