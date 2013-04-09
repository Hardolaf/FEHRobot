#include "robotnormal.h"
#include <FEHServo.h>

#ifndef ROBOTDEBUG_H
#define ROBOTDEBUG_H

/**
 * RobotDebug represents a robot that is currently being tested or calibrated. A
 * robot of this type need not be calibrated and may ignore calibration values
 * at any time.
 */
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
    void testServoElevatorSetAngle();
};

#endif // ROBOTDEBUG_H
