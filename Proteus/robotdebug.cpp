#include "robotdebug.h"

RobotDebug::RobotDebug()
{
    RobotNormal::setup(false);
}

RobotDebug::RobotDebug(bool calibrate)
{
    RobotNormal::setup(calibrate);
}
