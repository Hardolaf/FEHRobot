#include "robot.h"
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHServo.h>

#ifndef ROBOTNORMAL_H
#define ROBOTNORMAL_H

class RobotNormal : virtual public Robot
{
public:
    RobotNormal();

    void movementMotorManualSet(int speedLeft, int speedRight);
    void movementStraight(int speed, float distance);
    void movementLeft(int angle);
    void movementRight(int angle);
    int movementEncoderCountLeft();
    int movementEncoderCountRight();
    void movementEncoderCountReset();

    void motorSAMOpen();
    void motorSAMClose();

    bool lightSensorSeeStart();

    bool bumpSwitchFrontEitherPressed();
    bool bumpSwitchFrontLeftPressed();
    bool bumpSwitchFrontRightPressed();
    bool bumpSwitchFrontBothPressed();

    bool bumpSwitchBackEitherPressed();
    bool bumpSwitchBackLeftPressed();
    bool bumpSwitchBackRightPressed();
    bool bumpSwitchBackBothPressed();

    bool buttonRightPressed();
    bool buttonMiddlePressed();
    bool buttonLeftPressed();

    int optosensorLeftSeesLine();
    int optosensorMiddleSeesLine();
    int optosensorRightSeesLine();
protected:
    void setup(bool);
    void calibrate();

    /*! Left motor */
    FEHMotor* motorLeft;
    /*! Right motor */
    FEHMotor* motorRight;
    /*! SAM motor */
    FEHMotor* motorSAM;

    /*! Left Encoder */
    FEHEncoder* encoderLeft;
    /*! Right encoder */
    FEHEncoder* encoderRight;

    /*! Elevator servo motor */
    FEHServo* elevator;

    /*! Light sensor (CDS cell) */
    AnalogInputPin* lightSensor;
    /*! Left analog optosensor */
    AnalogInputPin* optosensorLeft;
    /*! Middle analog optosensor */
    AnalogInputPin* optosensorMiddle;
    /*! Right analog optosensor */
    AnalogInputPin* optosensorRight;

    /*! Buttons */
    ButtonBoard* buttons;

    /*! Front right bump switch */
    DigitalInputPin* bumpSwitchFrontRight;
    /*! Front left bump switch */
    DigitalInputPin* bumpSwitchFrontLeft;
    /*! Back right bump switch */
    DigitalInputPin* bumpSwitchBackRight;
    /*! Back left bump switch */
    DigitalInputPin* bumpSwitchBackLeft;
};

#endif // ROBOTNORMAL_H
