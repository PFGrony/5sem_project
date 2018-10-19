#include "computerVision.h"

///Sensor Data
//If lock==0 data containers is empty
static bool lidarLock=0;
static bool cameraLock=0;
//Camera
static cv::Mat matCamera;
//Lidar
static std::array<float,200> lidarAngle;
static std::array<float,200> lidarRange;

//time
static int nsecCopy;
static int secCopy;

computerVision::computerVision()
{
}

// Locks
bool computerVision::getLidarLock()
{
    return lidarLock;
}
bool computerVision::getCameraLock()
{
    return cameraLock;
}

// Get Data containers
cv::Mat computerVision::getMatCamera()
{
    return matCamera;
}

std::array<float,200> computerVision::getLidarAngle()
{
    return lidarAngle;
}
std::array<float,200> computerVision::getLidarRange()
{
    return lidarRange;
}

bool computerVision::getCircleBool()
{
    return circle_bool;
}

int computerVision::getOffset()
{
    return offset;
}



void computerVision::seeLidar()
{
    if(lidarLock==1)
    {
        //Show Lidar camera
        cv::Mat im(400, 400, CV_8UC3);
        im.setTo(0);

        for (int i = 0; i < 200; i++)
        {
            float angle = lidarAngle.at(i);
            float range = lidarRange.at(i);
            float range_min = 0.08;
            float range_max = 10;
            float px_per_m = 200 / range_max;   //20



            //    double intensity = msg->scan().intensities(i);
            cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                                200.5f - range_min * px_per_m * std::sin(angle));
            cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                              200.5f - range * px_per_m * std::sin(angle));
            cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
                     cv::LINE_AA, 4);

        }

        cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
        cv::putText(im, std::to_string(secCopy) + ":" + std::to_string(nsecCopy),
                    cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
                    cv::Scalar(255, 0, 0));
        cv::imshow("Lidar",im);
    }
}

void computerVision::seeCamera()
{
    if(cameraLock==1)
    {
        cv::imshow("newCamera",matCamera);
    }
}


void computerVision::seeCameraNew()
{
    if(cameraLock==1)
    {

        //int(height) 240
        //int(width) 320

        offset = 160;
        int rad = 0;
        cv::Mat im;
        im=matCamera.clone();
        cv::cvtColor(im, im, CV_RGB2BGR);
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
                cv::Vec3i c = circles.at(i);
                cv::Point center = cv::Point(c[0], c[1]);

                if (abs(int(c[0])-160)<abs(offset) && int(c[2]) > rad)
                {
                    offset = int(c[0])-160;
                    rad = int(c[2]);
                    //std::cout << "off: " << offset << ", rad: " << rad <<  std::endl;
                }

                //             circle center
                cv::circle( im, center, 1, cv::Scalar(255,0,0), 3, cv::LINE_AA);
                //             circle outline
                int radius = c[2];
                cv::circle( im, center, radius, cv::Scalar(255,0,0), 3, cv::LINE_AA);
            }
        }

        im = im.clone();
        cv::cvtColor(im, im, CV_BGR2RGB);

        cv::imshow("cameraIMPROVED", im);
    }


}


void computerVision::seeLidarNew()
{

    if(lidarLock==1)
    {
        //Show Lidar camera
        cv::Mat im(400, 400, CV_8UC3);
        im.setTo(0);

        for (int i = 0; i < 200; i++)
        {
            float angle = lidarAngle.at(i);
            float range = lidarRange.at(i);
            float range_min = 0.08;
            float range_max = 10;
            float px_per_m = 200 / range_max;   //20


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

        }

        cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
        cv::putText(im, std::to_string(secCopy) + ":" + std::to_string(nsecCopy),
                    cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
                    cv::Scalar(255, 0, 0));


        cv::imshow("LidarIMPROVED",im);
    }
}




























//Gazebo functinality

void computerVision::cameraCallback(ConstImageStampedPtr &msg)
{
    std::size_t width = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    im = im.clone();
    cv::cvtColor(im, im, CV_BGR2RGB);

    cameraLock=1;
    matCamera=im.clone();

    //    mutex.lock();
    //    cv::imshow("camera", im);
    //    mutex.unlock();
}

void computerVision::lidarCallback(ConstLaserScanStampedPtr &msg)
{

    //  std::cout << ">> " << msg->DebugString() << std::endl;
    float angle_min = float(msg->scan().angle_min());
    //  double angle_max = msg->scan().angle_max();
    float angle_increment = float(msg->scan().angle_step());

    float range_min = float(msg->scan().range_min());   //0.08
    float range_max = float(msg->scan().range_max());   //10

    int sec = msg->time().sec();
    int nsec = msg->time().nsec();

    int nranges = msg->scan().ranges_size();
    int nintensities = msg->scan().intensities_size();

    assert(nranges == nintensities);

    int width = 400;
    int height = 400;
    float px_per_m = 200 / range_max;   //20

    //    cv::Mat im(height, width, CV_8UC3);
    //    im.setTo(0);


    for (int i = 0; i < nranges; i++)
    {
        float angle = angle_min + i * angle_increment;
        float range = std::min(float(msg->scan().ranges(i)), range_max);


        lidarLock=1;
        lidarRange.at(i)=range;
        lidarAngle.at(i)=angle;



        //        //    double intensity = msg->scan().intensities(i);
        //        cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
        //                            200.5f - range_min * px_per_m * std::sin(angle));
        //        cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
        //                          200.5f - range * px_per_m * std::sin(angle));
        //        cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
        //                 cv::LINE_AA, 4);

    }

    nsecCopy=nsec;
    secCopy=sec;

    //    cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
    //    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
    //                cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
    //                cv::Scalar(255, 0, 0));

    //    mutex.lock();
    //    cv::imshow("lidar", im);
    //    mutex.unlock();
}

void computerVision::startCamera(gazebo::transport::NodePtr &node)
{
    cameraSubscriber = node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);
}

void computerVision::startLidar(gazebo::transport::NodePtr &node)
{
    lidarSubscriber= node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);
}



