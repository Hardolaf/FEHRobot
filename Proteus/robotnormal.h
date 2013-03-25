#include "robot.h"
#include <FEHMotor.h>

#ifndef ROBOTNORMAL_H
#define ROBOTNORMAL_H

class RobotNormal : public Robot
{
public:
    RobotNormal();
    void movementStraight(int, float);
    void movementLeft(int);
    void movementRight(int);
protected:
    void setup();

    /*! Left motor */
    FEHMotor leftMotor;
    /*! Right motor */
    FEHMotor rightMotor;
    /*! Elevator servo motor */
    int elevatorServoMotor;

};

#endif // ROBOTNORMAL_H
