#ifndef FUZZYCONTROLLER_H
#define FUZZYCONTROLLER_H

#include "fl/Headers.h"

class fuzzyController
{
public:
    fuzzyController();

    void fuzzyInit();
    void fuzzyUpdate(float array[]);
    float getSpeed();
    float getSteer();
private:
    float steer;
    float speed;

    float farLeft;
    float left;
    float forward;
    float right;
    float farRight;

    fl::Engine* engine;

    fl::InputVariable* inFarLeft;
    fl::InputVariable* inLeft;
    fl::InputVariable* inForward;
    fl::InputVariable* inRight;
    fl::InputVariable* inFarRight;

    fl::OutputVariable* outSteer;
    fl::OutputVariable* outSpeed;

    fl::RuleBlock* mamdani;
};

#endif // FUZZYCONTROLLER_H
