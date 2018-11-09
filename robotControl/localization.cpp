#include "localization.h"

localization::localization()
{

}

localization::localization(const std::string mapPath)
{
    map = cv::imread(mapPath,CV_LOAD_IMAGE_GRAYSCALE);
    cv::resize(map,map,cv::Size(),resizeFactor,resizeFactor,cv::INTER_NEAREST);
}

void localization::setRobPos(double x, double y, double a)
{
    robotX = (map.cols/2)+x*resizeFactor*1.4; // underligt offset bliver gjort op for med 1.4
    robotY = (map.rows/2)-y*resizeFactor*1.4;
    robotA = a;
}

void localization::updateRobPos(float *array, double speed, double steer)
{
    double oldrobotPosX=robotPosX;
    double oldrobotPosY=robotPosY;

    double radChange=steer*0.01;
    double lengthTraveled=speed*0.01*resizeFactor; // speed * time step * size

    robotA -= radChange;

    robotX=lengthTraveled*cos(robotA)+oldrobotPosX;
    robotY=lengthTraveled*sin(robotA)+oldrobotPosY;

    double checkX = 0;
    double checkY = 0;
    double angleBetweenLidar = 0.0227;
    double angleZeroLidar = -2.27;
    for(int i=0; i<200;i++)
    {
        if(0.10 < *(array+i) && *(array+i) < 8.00)
        {
            checkX = robotX+(resizeFactor**(arrays+i)*cos(robotA+angleZeroLidar+angleBetweenLidar*i));
            checkY = robotY+(resizeFactor**(arrays+i)*sin(robotA+angleZeroLidar+angleBetweenLidar*i));
        }
    }
}

double localization::getRobX()
{
    return (robotX/(resizeFactor*1.4))-(map.cols/2);
}

double localization::getRobY()
{
    return (robotY/(resizeFactor*1.4))+(map.rows/2);
}

double localization::getRobA()
{
    return robotA;
}
