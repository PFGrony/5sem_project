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
    void calculateObstaclePoints(float* arrays);
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

    int width = 250;
    int height = 250;
    cv::Mat1b im_map;

    int size_update = 0;

    std::vector<int> currObstaclePointsX;
    std::vector<int> currObstaclePointsY;
};

#endif // GENERATEMAP_H
