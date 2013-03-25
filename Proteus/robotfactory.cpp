#include "robotfactory.h"

RobotFactory::RobotFactory()
{
}

Robot* RobotFactory::Create(int mode) {
    switch (mode) {
    case RUN_MODE_DEBUG:
        // Create debug robot
        break;
    case RUN_MODE_NORMAL:
    default:
        // Create normal operation robot
        break;
    }
}
