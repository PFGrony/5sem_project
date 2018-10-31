#include "generateMap.h"

generateMap::generateMap()
{
        im_map=cv::Mat1b(height, width, 255);
        robotPosX = width/2;
        robotPosY = height/2;
}

void generateMap::calculateRobotPos(double speed, double steer)
{
    double oldrobotPosX=robotPosX;
    double oldrobotPosY=robotPosY;
    double radChange=steer*0.01;

    double lengthTraveled=speed*0.01;
    robotPosX=lengthTraveled*cos(radChange)+oldrobotPosX;
    robotPosY=lengthTraveled*sin(radChange)+oldrobotPosY;

    robotOrien+=radChange;
}

void generateMap::calculateObstaclePoints(std::array<float, 200> arrays)
{
    double angleBetweenLidar = 0.0226899;
    double angleZeroLidar = -2.26899;
    for(int i=0; i<200;i++)
    {
        if(0.08 < arrays.at(i) < 10)
        {
            currObstaclePointsX.push_back(robotPosX+(10*arrays.at(i)*cos(angleZeroLidar+angleBetweenLidar*i)));
            currObstaclePointsY.push_back(robotPosY+(10*arrays.at(i)*sin(angleZeroLidar+angleBetweenLidar*i)));
        }
    }
}

void generateMap::insertPointsOnMap()
{
    //Noget til at indsætte punkter fra   currObstaclePointsX og  currObstaclePointsY
    //Robotten starter i det den kalder 0.0, men dette er jo ikke pixel (x,y)=(0,0), find dette offset.




    for(; currObstaclePointsX.size() > 0;)
    {
        if(abs(currObstaclePointsX.back()) > width || abs(currObstaclePointsY.back()) > height)
        {
            height=2*height;
            width=2*width;
            cv::Mat1b im_newmap(height,width,1);
            im_map.copyTo(im_newmap(cv::Rect(height*0.25,width*0.25,im_map.cols,im_map.rows)));
            im_map=im_newmap.clone();
            robotPosX += width/2;
            robotPosY += height/2;

        }


        im_map.at<uchar>(currObstaclePointsX.back(),currObstaclePointsY.back())=0;
        currObstaclePointsX.pop_back();
        currObstaclePointsY.pop_back();
    }
//    cv::imshow("Map",im_map);


}

void generateMap::setRobPos(double x, double y, double a)
{
    robotPosX = x;
    robotPosY = y;
    robotOrien = a;
}
