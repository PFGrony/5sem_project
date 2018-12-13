#ifndef FUZZYCONTROLLER_H
#define FUZZYCONTROLLER_H

#include "fl/Headers.h"

class fuzzyController
{
public:
    fuzzyController();

    void fuzzyInit();
    void fuzzyUpdate(float* arrays,double robot_x,double robot_y,double robot_a, double point_x, double point_y);
    float getSpeed();
    float getSteer();
private:
    float steer = 0;
    float speed = 0;

    float farLeft;
    float left;
    float forward;
    float right;
    float farRight;

    float goal;

    fl::Engine* engine;

    fl::InputVariable* inFarLeft;
    fl::InputVariable* inLeft;
    fl::InputVariable* inForward;
    fl::InputVariable* inRight;
    fl::InputVariable* inFarRight;
    fl::InputVariable* inAngle;

    fl::OutputVariable* outSteer;
    fl::OutputVariable* outSpeed;

    fl::RuleBlock* mamdani;
};

#endif // FUZZYCONTROLLER_H