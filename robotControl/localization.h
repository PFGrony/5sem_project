#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "opencv2/opencv.hpp"
#include <cmath>

class localization
{
public:
    localization();
    localization(const std::string mapPath);

    void setRobPos(double x, double y, double a);

    void updateRobPos(float* array, double speed, double steer);

    double getRobX();
    double getRobY();
    double getRobA();

private:
    double robotX = 0;
    double robotY = 0;
    double robotA = 0;

    int resizeFactor = 4;

    cv::Mat map;
};

#endif // LOCALIZATION_H
