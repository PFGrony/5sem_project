#include "computerVision.h"

///Sensor Data
//If lock==0 data containers is empty
static bool lidarLock=0;
static bool cameraLock=0;
//Camera
static cv::Mat matCamera;
//Lidar
static float lidarRange[200] = {};
static float *lR = lidarRange;

static float lidarAngle[200] = {};
static float *lA = lidarAngle;

//time
static int nsecCopy;
static int secCopy;

computerVision::computerVision()
{
    templ=cv::imread("../robotControl/images/circle3.png",CV_LOAD_IMAGE_ANYCOLOR);
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

float* computerVision::getLidarAngle()
{
    return lA;
}
float* computerVision::getLidarRange()
{
    return lR;
}

bool computerVision::getCircleBool()
{
    return circle_bool;
}

int computerVision::getOffset()
{
    return offset;
}

float computerVision::getRadius()
{
    return ballRadius;
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
            float angle = *(lA+i);
            float range = *(lR+i);
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
        cv::imshow("Camera",matCamera);
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

        cv::Mat gray;
        cv::cvtColor(im, gray, cv::COLOR_BGR2GRAY);
        cv::medianBlur(gray, gray, 5);
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray,circles, cv::HOUGH_GRADIENT,1,gray.rows/8,50,20,0,0);

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

        std::cout<<offset<<std::endl;
        im = im.clone();
        cv::cvtColor(im, im, CV_BGR2RGB);
        cv::imshow("Camera", im);
    }


}

//Hough+Canny
void computerVision::seeCameraV2()
{
    if(cameraLock==1)
    {



        cv::Mat color;
        color=matCamera.clone();

        cv::Mat rgb[3];   //destination array
        cv::split(color,rgb);//split source

        for(int i=0;i<rgb[2].rows;i++)
        {
            for(int j=0;j<rgb[2].cols;j++)
            {
                if(rgb[2].at<uchar>(i,j)>10)
                    rgb[2].at<uchar>(i,j)=255;
            }
        }

        //Note: OpenCV uses BGR color order
        //        cv::imshow("blue.png",rgb[2]); //blue channel


        //                cv::namedWindow("input"); cv::imshow("input", color);


        cv::Mat gray;
        cv::cvtColor(color, gray, CV_RGB2GRAY);


        cv::Mat canny;
        //edge detection
        cv::Canny(gray, canny, 200,20);
        canny=rgb[2].clone();
//        cv::namedWindow("canny2"); cv::imshow("canny2", canny>0);

        std::vector<cv::Vec3f> circles;

        // Apply the Hough Transform to find the circles
        cv::HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 1, 60, 50, 20, 1, 100 );


        int rad=0;
        int newrad = 0;
        // Draw the circles detected
        for( size_t i = 0; i < circles.size(); i++ )
        {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

            newrad = cvRound(circles[i][2]);

            //Hvad er dette kuus??
            if(newrad != newrad)
                newrad = 0;

            if (newrad > rad)
            {
                rad = newrad;
                offset = int(circles[i][0])-160;
            }

            cv::circle( color, center, 3, cv::Scalar(0,255,255), -1);
            cv::circle( color, center, newrad, cv::Scalar(0,0,255), 1 );
        }

        //Info
        ballRadius=rad;

        if (circles.size() > 0)
            circle_bool = 1;
        else
            circle_bool = 0;

        //        std::cout<<"Circle: "<<circle_bool<<"\toffset: "<<offset<<std::endl;

        ///See circle percentage match

        //        //compute distance transform:
        //        cv::Mat dt;
        //        cv::distanceTransform(255-(canny>0), dt, CV_DIST_L2 ,3);
        //        //        cv::namedWindow("distance transform"); cv::imshow("distance transform", dt/255.0f);

        //        // test for semi-circles:
        //        float minInlierDist = 2.0f;
        //        for( size_t i = 0; i < circles.size(); i++ )
        //        {
        //            // test inlier percentage:
        //            // sample the circle and check for distance to the next edge
        //            unsigned int counter = 0;
        //            unsigned int inlier = 0;

        //            cv::Point2f center((circles[i][0]), (circles[i][1]));
        //            float radius = (circles[i][2]);
        //            // maximal distance of inlier might depend on the size of the circle
        //            float maxInlierDist = radius/25.0f;
        //            if(maxInlierDist<minInlierDist)
        //                maxInlierDist = minInlierDist;

        //            //TODO: maybe paramter incrementation might depend on circle size!
        //            for(float t =0; t<2*3.14159265359f; t+= 0.1f)
        //            {
        //                counter++;
        //                float cX = radius*cos(t) + circles[i][0];
        //                float cY = radius*sin(t) + circles[i][1];

        //                if(dt.at<float>(cY,cX) < maxInlierDist)
        //                {
        //                    inlier++;
        ////                    cv::circle(color, cv::Point2i(cX,cY),3, cv::Scalar(0,255,0));
        //                }
        ////                else
        ////                    cv::circle(color, cv::Point2i(cX,cY),3, cv::Scalar(255,0,0));
        //            }
        //            std::cout << 100.0f*(float)inlier/(float)counter << " % of a circle with radius " << radius << " detected" << std::endl;
        //        }

//                cv::imshow("blue",rgb[2]);
        cv::namedWindow("Camera"); cv::imshow("Camera", color);

    }
}

//void computerVision::seeCameraV3()
//{
//}


void computerVision::seeLidarNew()
{

    if(lidarLock==1)
    {
        //Show Lidar camera
        cv::Mat im(400, 400, CV_8UC3);
        im.setTo(0);

        for (int i = 0; i < 200; i++)
        {
            float angle = *(lA+i);
            float range = *(lR+i);
            float range_min = 0.08;
            float range_max = 10;
            float px_per_m = 200 / range_max;   //20


            cv::Scalar color;
            if (i<60 || i>140)
                color = cv::Scalar(0,255,255);
            if (i>59 && i<141)
                color = cv::Scalar(0,255,0);
            if (i>84 && i<116)
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


        cv::imshow("LIDAR",im);
    }
}

void computerVision::templateMatching()
{
    if(cameraLock==1)
    {
        cv::Mat img,imgDisplay;
        img=matCamera.clone();
        img.copyTo(imgDisplay);
        /// Create the result matrix
        int resultCols =  img.cols - templ.cols + 1;
        int resultRows = img.rows - templ.rows + 1;

        cv::Mat result(resultRows, resultCols, CV_32FC1 );

        int matchMethod=CV_TM_CCORR_NORMED;
        /// Do the Matching and Normalize
        cv::matchTemplate( img, templ, result, matchMethod );


        //        cv::Mat dilated, thresholdedMatchingSpace,localMaxima,thresholded8bit;
        //        cv::dilate(result,dilated,cv::Mat());
        //        cv::compare(result,dilated,localMaxima,cv::CMP_EQ);
        //        double threshold=0;
        //        cv::threshold(result,thresholdedMatchingSpace,threshold,255,cv::THRESH_BINARY);
        //        thresholdedMatchingSpace.convertTo(thresholded8bit,CV_8U);
        //        cv::bitwise_and(localMaxima,thresholded8bit,localMaxima);




        cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

        /// Localizing the best match with minMaxLoc
        double minVal, maxVal;
        cv::Point minLoc,maxLoc, matchLoc;

        cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

        /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
        if( matchMethod  == CV_TM_SQDIFF || matchMethod == CV_TM_SQDIFF_NORMED )
        { matchLoc = minLoc; }
        else
        { matchLoc = maxLoc; }

        cv::rectangle( imgDisplay, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );
        cv::rectangle( result, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );

        //          cv::imshow("thresh",localMaxima);



        cv::imshow( "image_window", imgDisplay );
        //        cv::imshow( "result_window", result );


        return;
    }
}

void computerVision::haarClassifier()
{
    if(cameraLock==1)
    {
        cv::CascadeClassifier cascade;

        cv::Mat color;
        color=matCamera.clone();
        cv::Mat rgb[3];   //destination array
        cv::split(color,rgb);//split source

        for(int i=0;i<rgb[2].rows;i++)
        {
            for(int j=0;j<rgb[2].cols;j++)
            {
                if(rgb[2].at<uchar>(i,j)>10)
                    rgb[2].at<uchar>(i,j)=255;
            }
        }
        cascade.load("../robotControl/classifiers/classifiers3.xml");
        if(!cascade.empty())
        {
            std::vector<cv::Rect> circles;

            cv::equalizeHist( rgb[2], rgb[2] );
            cascade.detectMultiScale( rgb[2], circles, 1.1, 10,CV_HAAR_SCALE_IMAGE,cv::Size(30,30));


            for( size_t  i= 0; i < circles.size(); i++ )
            {
                cv::Point center( circles[i].x+ circles[i].width*0.5, circles[i].y + circles[i].height*0.5 );
                int radius = cvRound( (circles[i].width + circles[i].height)*0.25 );
                cv::circle( color, center, radius, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );
            }




        }

        cv::imshow("color",color);
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
}

void computerVision::lidarCallback(ConstLaserScanStampedPtr &msg)
{
    float angle_min = float(msg->scan().angle_min());
    float angle_increment = float(msg->scan().angle_step());

    //float range_min = float(msg->scan().range_min());   //0.08
    float range_max = float(msg->scan().range_max());   //10

    int sec = msg->time().sec();
    int nsec = msg->time().nsec();

    int nranges = msg->scan().ranges_size();
    int nintensities = msg->scan().intensities_size();

    assert(nranges == nintensities);


    for (int i = 0; i < nranges; i++)
    {
        float angle = angle_min + i * angle_increment;
        float range = std::min(float(msg->scan().ranges(i)), range_max);
        lidarLock=1;
        *(lR+i)=range;
        *(lA+i)=angle;
    }
    nsecCopy=nsec;
    secCopy=sec;
}

void computerVision::startCamera(gazebo::transport::NodePtr &node)
{
    cameraSubscriber = node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);
}

void computerVision::startLidar(gazebo::transport::NodePtr &node)
{
    lidarSubscriber= node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);
}



