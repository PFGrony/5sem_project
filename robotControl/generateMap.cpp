#include "generateMap.h"

generateMap::generateMap()
{
    cv::Mat1b im_newmap(height,width,255);
    im_map=im_newmap.clone();
    robotPosX = width/2;
    robotPosY = height/2;
}

void generateMap::calculateRobotPos(double speed, double steer)
{
    double oldrobotPosX=robotPosX;
    double oldrobotPosY=robotPosY;

    double radChange=steer*0.01;
    double lengthTraveled=speed*0.01*5; // speed * time step * size

    robotOrien -= radChange;

    robotPosX=lengthTraveled*cos(robotOrien)+oldrobotPosX;
    robotPosY=lengthTraveled*sin(robotOrien)+oldrobotPosY;
}

void generateMap::calculateObstaclePoints(float *arrays)
{
    while(currObstaclePointsX.size() > 0)
    {
        currObstaclePointsX.pop_back();
        currObstaclePointsY.pop_back();
    }

    double angleBetweenLidar = 0.0226899;
    double angleZeroLidar = -2.26899;
    for(int i=0; i<200;i++)
    {
        if(0.10 < *(arrays+i) && *(arrays+i) < 9.00)
        {
            currObstaclePointsX.push_back(robotPosX+(5**(arrays+i)*cos(robotOrien+angleZeroLidar+angleBetweenLidar*i)));
            currObstaclePointsY.push_back(robotPosY+(5**(arrays+i)*sin(robotOrien+angleZeroLidar+angleBetweenLidar*i)));
        }
    }
}

void generateMap::insertPointsOnMap()
{
    // kuus hack - glitch fiks, hvor der var nogen "phantom" linjer rundt omkring, hvor de ikke burde være
    if (size_update < 4)
    {
        size_update++;
        return;
    }
    //Noget til at indsætte punkter fra   currObstaclePointsX og  currObstaclePointsY
    //Robotten starter i det den kalder 0.0, men dette er jo ikke pixel (x,y)=(0,0), find dette offset.

    for(; currObstaclePointsX.size() > 0;)
    {
        if(currObstaclePointsX.back() > width-5 || currObstaclePointsY.back() > height-5 || currObstaclePointsY.back() < 5 || currObstaclePointsX.back() < 5)
        {
            height=2*height;
            width=2*width;
            cv::Mat1b im_newmap(height,width,255);
            im_map.copyTo(im_newmap(cv::Rect(height*0.25,width*0.25,im_map.cols,im_map.rows)));
            im_map=im_newmap.clone();

            robotPosX += width/2;
            robotPosY += height/2;
            size_update = 0;
            break;
        }


        im_map.at<uchar>(currObstaclePointsX.back(),currObstaclePointsY.back())=0;
        currObstaclePointsX.pop_back();
        currObstaclePointsY.pop_back();
    }
    cv::imshow("Map",im_map);


}

void generateMap::setRobPos(double x, double y, double a)
{
    robotPosX = (x*5)+height/2;
    robotPosY = (y*5)+width/2;
    robotOrien = a;
}

double generateMap::getXPos()
{
    return (robotPosX - width/2)/5;
}

double generateMap::getYPos()
{
    return (robotPosY - height/2)/5;
}

double generateMap::getAngle()
{
   return robotOrien;
}
