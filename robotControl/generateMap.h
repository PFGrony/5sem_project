#ifndef GENERATEMAP_H
#define GENERATEMAP_H

#include "opencv2/opencv.hpp"

#include <math.h>
#include <array>
#include <vector>

class generateMap
{
public:
    generateMap();
    void calculateRobotPos(double speed, double steer);
    void calculateObstaclePoints(std::array<float,200> arrays);
    void insertPointsOnMap();

    void setRobPos(double x, double y, double a);

    cv::Mat getMat()
    {
        return im_map;
    }

private:
    double robotPosX;
    double robotPosY;
    double robotOrien=0;

    int width = 500;
    int height = 500;
    cv::Mat1b im_map;

    std::vector<int> currObstaclePointsX;
    std::vector<int> currObstaclePointsY;
};

#endif // GENERATEMAP_H
