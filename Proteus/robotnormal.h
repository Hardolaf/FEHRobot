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

};

#endif // ROBOTNORMAL_H
