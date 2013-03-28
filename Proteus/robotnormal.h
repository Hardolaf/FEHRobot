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
    void movementStraight(int, float);
    void movementLeft(int);
    void movementRight(int);
    bool lightSensorSeeStart();
    bool bumpSwitch
protected:
    void setup(bool);
    void calibrate();

    /*! Left motor */
    FEHMotor* leftMotor;
    /*! Right motor */
    FEHMotor* rightMotor;

    /*! Left Encoder */
    FEHEncoder* leftEncoder;
    /*! Right encoder */
    FEHEncoder* rightEncoder;

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
    DigitalInputPin bumpSwitchFrontRight;
    /*! Front left bump switch */
    DigitalInputPin bumpSwitchFrontLeft;
    /*! Back right bump switch */
    DigitalInputPin bumpSwitchBackRight;
    /*! Back left bump switch */
    DigitalInputPin bumpSwitchBackLeft;
};

#endif // ROBOTNORMAL_H
