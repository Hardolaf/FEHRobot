#include "robotnormal.h"
#include <FEHServo.h>

#ifndef ROBOTDEBUG_H
#define ROBOTDEBUG_H

class RobotDebug : public RobotNormal
{
public:
    RobotDebug();
    RobotDebug(bool);
    void calibrateEncoders();
    void calibrateOptosensors();
    void calibrateLightSensor();
    void calibrateServoElevator();
    void calibrateServoArm();

    void testMovementForward();
    void testMotorSAM();
protected:
    void calibrateServo(FEHServo servo);
};

#endif // ROBOTDEBUG_H
