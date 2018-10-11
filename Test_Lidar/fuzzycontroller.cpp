#include "fuzzycontroller.h"

fuzzyController::fuzzyController()
{

}

void fuzzyController::fuzzyInit()
{
    using namespace fl;

    engine = new Engine;
    engine->setName("ObstacleAvoidance");
    engine->setDescription("");

    float mid = 1.500;
    float close = 1.000;
    float far = 3.000;

    inFarLeft = new InputVariable;
    inFarLeft->setName("inFarLeft");
    inFarLeft->setDescription("");
    inFarLeft->setEnabled(true);
    inFarLeft->setRange(0.000, 10.000);
    inFarLeft->setLockValueInRange(false);
    inFarLeft->addTerm(new Ramp("close", mid, close));
    inFarLeft->addTerm(new Triangle("mid",close,mid,far));
    inFarLeft->addTerm(new Ramp("far",mid,far));
    engine->addInputVariable(inFarLeft);

    inLeft = new InputVariable;
    inLeft->setName("inLeft");
    inLeft->setDescription("");
    inLeft->setEnabled(true);
    inLeft->setRange(0.000, 10.000);
    inLeft->setLockValueInRange(false);
    inLeft->addTerm(new Ramp("close", mid, close));
    inLeft->addTerm(new Triangle("mid",close,mid,far));
    inLeft->addTerm(new Ramp("far",mid,far));
    engine->addInputVariable(inLeft);

    inForward = new InputVariable;
    inForward->setName("inForward");
    inForward->setDescription("");
    inForward->setEnabled(true);
    inForward->setRange(0.000, 10.000);
    inForward->setLockValueInRange(false);
    inForward->addTerm(new Ramp("close", mid, close));
    inForward->addTerm(new Triangle("mid",close,mid,far));
    inForward->addTerm(new Ramp("far",mid,far));
    engine->addInputVariable(inForward);

    inRight = new InputVariable;
    inRight->setName("inRight");
    inRight->setDescription("");
    inRight->setEnabled(true);
    inRight->setRange(0.000, 10.000);
    inRight->setLockValueInRange(false);
    inRight->addTerm(new Ramp("close", mid, close));
    inRight->addTerm(new Triangle("mid",close,mid,far));
    inRight->addTerm(new Ramp("far",mid,far));
    engine->addInputVariable(inRight);

    inFarRight = new InputVariable;
    inFarRight->setName("inFarRight");
    inFarRight->setDescription("");
    inFarRight->setEnabled(true);
    inFarRight->setRange(0.000, 10.000);
    inFarRight->setLockValueInRange(false);
    inFarRight->addTerm(new Ramp("close", mid, close));
    inFarRight->addTerm(new Triangle("mid",close,mid,far));
    inFarRight->addTerm(new Ramp("far",mid,far));
    engine->addInputVariable(inFarRight);

    float sharpLeft = -1.000;
    float softLeft = -0.250;
    float softRight = 0.250;
    float sharpRight = 1.000;

    outSteer = new OutputVariable;
    outSteer->setName("outSteer");
    outSteer->setDescription("");
    outSteer->setEnabled(true);
    outSteer->setRange(-1.000, 1.000);
    outSteer->setLockValueInRange(false);
    outSteer->setAggregation(new Maximum);
    outSteer->setDefuzzifier(new Centroid(100));
    outSteer->setDefaultValue(fl::nan);
    outSteer->setLockPreviousValue(false);
    outSteer->addTerm(new Ramp("sharpLeft",softLeft,sharpLeft));
    outSteer->addTerm(new Triangle("softLeft", sharpLeft, softLeft,softRight));
    outSteer->addTerm(new Triangle("softRight", softLeft, softRight,sharpRight));
    outSteer->addTerm(new Ramp("sharpRight",softRight,sharpRight));
    engine->addOutputVariable(outSteer);

    float slow = 0.000;
    float medium = 0.500;
    float fast = 1.000;

    outSpeed = new OutputVariable;
    outSpeed->setName("outSpeed");
    outSpeed->setDescription("");
    outSpeed->setEnabled(true);
    outSpeed->setRange(0.000, 1.000);
    outSpeed->setLockValueInRange(false);
    outSpeed->setAggregation(new Maximum);
    outSpeed->setDefuzzifier(new Centroid(100));
    outSpeed->setDefaultValue(fl::nan);
    outSpeed->setLockPreviousValue(false);
    outSpeed->addTerm(new Ramp("slow", medium, slow));
    outSpeed->addTerm(new Triangle("medium",slow,medium,fast));
    outSpeed->addTerm(new Ramp("fast",medium,fast));
    engine->addOutputVariable(outSpeed);

    mamdani = new RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new Minimum);
    mamdani->setDisjunction(new Maximum);
    mamdani->setImplication(new Minimum);
    mamdani->setActivation(new General);
    mamdani->addRule(Rule::parse("if obstacle is left then mSteer is right", engine));
    mamdani->addRule(Rule::parse("if obstacle is right then mSteer is left", engine));
    engine->addRuleBlock(mamdani);

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);
}

void fuzzyController::fuzzyUpdate(float array[])
{
    farLeft = 10.0;
    left = 10.0;
    forward = 10.0;
    right = 10.0;
    farRight = 10.0;

    int rFarLeft = 40;
    int rLeft = 80;
    int rForward = 120;
    int rRight = 160;
    int rFarRight = 200;

    for (int i = 0;i<200;i++)
    {
        if (i<rFarLeft)
            if (farLeft > array[i])
                farLeft = array[i];

        if (i > (rFarLeft-1) && i<rLeft)
            if (left > array[i])
                left = array[i];

        if (i > (rLeft-1) && i<rForward)
            if (forward > array[i])
                forward = array[i];

        if (i > (rForward-1) && i<rRight)
            if (right > array[i])
                right = array[i];

        if (i >(rRight-1) && i<rFarRight)
            if (farRight > array[i])
                farRight = array[i];
    }

    inFarLeft->setValue(farLeft);
    inLeft->setValue(left);
    inForward->setValue(forward);
    inRight->setValue(right);
    inFarRight->setValue(farRight);

    engine->process();

    speed = outSpeed->getValue();
    steer = outSteer->getValue();

}

float fuzzyController::getSpeed()
{
    return speed;
}

float fuzzyController::getSteer()
{
    return steer;
}
