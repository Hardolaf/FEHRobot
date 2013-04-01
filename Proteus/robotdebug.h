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
    void testServoElevator();
    void testServoArm();
};

#endif // ROBOTDEBUG_H
