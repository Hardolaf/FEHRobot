#include "robotnormal.h"

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

    void testMovementForward();
    void testMotorSAM();
};

#endif // ROBOTDEBUG_H
