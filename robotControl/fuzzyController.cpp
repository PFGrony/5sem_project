#include "fuzzyController.h"

fuzzyController::fuzzyController()
{

}

void fuzzyController::fuzzyInit()
{
    using namespace fl;

    engine = new Engine;
    engine->setName("ObstacleAvoidance");
    engine->setDescription("");

    inAngle = new InputVariable;
    inAngle->setName("inAngle");
    inAngle->setDescription("");
    inAngle->setEnabled(true);
    inAngle->setRange(-3.200, 3.200);
    inAngle->setLockValueInRange(false);
    inAngle->addTerm(new Ramp("left", 0.500, -0.500));
    inAngle->addTerm(new Ramp("right",-0.500, 0.500));
    engine->addInputVariable(inAngle);

    float close = 0.800;
    float mid = 1.600;
    float far = 2.800;

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

    float sharpLeft = -4.000;
    float softLeft = -0.500;
    float softRight = 0.500;
    float sharpRight = 4.000;

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
    mamdani->setConjunction(new AlgebraicProduct);
    mamdani->setDisjunction(new AlgebraicSum);
    mamdani->setImplication(new AlgebraicProduct);
    mamdani->setActivation(new General);
    //Path finding
    mamdani->addRule(Rule::parse("if inAngle is left and inLeft is not close then outSteer is sharpLeft", engine));
    mamdani->addRule(Rule::parse("if inAngle is right and inRight is not close then outSteer is sharpRight", engine));

    mamdani->addRule(Rule::parse("if inAngle is left and inForward is not close then outSteer is sharpLeft", engine));
    mamdani->addRule(Rule::parse("if inAngle is right and inForward is not close then outSteer is sharpRight", engine));
    //Steer
    mamdani->addRule(Rule::parse("if inLeft is close and inRight is not close then outSteer is sharpRight", engine));
    mamdani->addRule(Rule::parse("if inRight is close and inLeft is not close then outSteer is sharpLeft", engine));

    mamdani->addRule(Rule::parse("if inLeft is mid and inRight is not close then outSteer is sharpRight", engine));
    mamdani->addRule(Rule::parse("if inRight is mid and inLeft is not close then outSteer is sharpLeft", engine));

    mamdani->addRule(Rule::parse("if inLeft is far and inRight is not far then outSteer is softLeft", engine));
    mamdani->addRule(Rule::parse("if inRight is far and inRight is not far then outSteer is softRight", engine));

    mamdani->addRule(Rule::parse("if inFarLeft is far and inFarRight is mid then outSteer is softLeft", engine));
    mamdani->addRule(Rule::parse("if inFarRight is far and inFarLeft is mid then outSteer is softRight", engine));

    mamdani->addRule(Rule::parse("if inFarLeft is close and inFarRight is not close then outSteer is sharpRight", engine));
    mamdani->addRule(Rule::parse("if inFarRight is close and inFarLeft is not close then outSteer is sharpLeft", engine));

    mamdani->addRule(Rule::parse("if inForward is close and inFarLeft is not close then outSteer is sharpLeft",engine));
    mamdani->addRule(Rule::parse("if inForward is close and inFarRight is not close then outSteer is sharpRight",engine));

    //Speed
    mamdani->addRule(Rule::parse("if inForward is close then outSpeed is slow", engine));
    mamdani->addRule(Rule::parse("if inForward is mid then outSpeed is medium", engine));
    mamdani->addRule(Rule::parse("if inForward is far then outSpeed is fast", engine));
    engine->addRuleBlock(mamdani);

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);
}

void fuzzyController::fuzzyUpdate(float *arrays, double robot_x, double robot_y, double robot_a, double point_x, double point_y)
{
    double ahead_x = cos(robot_a);
    double ahead_y = sin(robot_a);

    double ac_x = point_x - robot_x;
    double ac_y = point_y - robot_y;

    goal =  acos(((ahead_x*ac_x)+(ahead_y*ac_y))/(sqrt(ahead_x*ahead_x+ahead_y*ahead_y)*sqrt(ac_x*ac_x+ac_y*ac_y)));

    if (goal != goal)
        goal = 0;

    double left_right = ahead_x*ac_y - ahead_y*ac_x;

    if(left_right < 0)
        goal *= -1;

    //std::cout << goal << std::endl;

    // setup to find the lowest value for left and right, and to find the averrage for the rest
    farLeft = 0;
    left = 10.0;
    forward = 10.0;
    right = 10.0;
    farRight = 0;
    // The boarder from each space
    int rFarLeft = 60;
    int rLeft = 85;
    int rForward = 115;
    int rRight = 140;
    int rFarRight = 200;

    for (int i = 0;i<200;i++)
    {
        if (i<rFarLeft)
            farLeft += *(arrays+i);

        if (i > (rFarLeft-1) && i<rLeft && left > *(arrays+i))
            left = *(arrays+i);

        if (i > (rLeft-1) && i<rForward && forward > *(arrays+i))
            forward = *(arrays+i);

        if (i > (rForward-1) && i<rRight && right > *(arrays+i))
            right = *(arrays+i);

        if (i >(rRight-1) && i<rFarRight)
            farRight += *(arrays+i);
    }

    farLeft = farLeft/(rFarLeft);
    //left = left/(rLeft-rFarLeft);
    //forward = forward/(rForward-rLeft);
    //right = right/(rRight-rForward);
    farRight = farRight/(rFarRight-rRight);

    // Load into engine, process it and set the new values for speed and steer
    inFarLeft->setValue(farLeft);
    inLeft->setValue(left);
    inForward->setValue(forward);
    inRight->setValue(right);
    inFarRight->setValue(farRight);
    inAngle->setValue(goal);


    engine->process();

    speed = outSpeed->getValue();
    steer = outSteer->getValue();
}

float fuzzyController::getSpeed()
{
    return 0.5*speed;
}

float fuzzyController::getSteer()
{
    // increased steer speed and making up for not knowing left and right
    return (-1*steer);
}
