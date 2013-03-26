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
    AnalogInputPin* leftOptosenor;
    /*! Middle analog optosensor */
    AnalogInputPin* middleOptosenor;
    /*! Right analog optosensor */
    AnalogInputPin* rightOptosenor;

    /*! Buttons */
    ButtonBoard* buttons;
};

#endif // ROBOTNORMAL_H
