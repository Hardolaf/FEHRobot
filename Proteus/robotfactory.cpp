#include "robotfactory.h"
#include "robot.h"
#include "robotsingleton.h"
#include "robotnormal.h"
#include "robotdebug.h"

RobotFactory::RobotFactory()
{
}

Robot* RobotFactory::Create(int mode) {
    if (mode == RobotFactory::RUN_MODE_DEBUG) {
        return new RobotDebug();
    } else {
        return new RobotNormal();
    }
}
