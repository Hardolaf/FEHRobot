#include "robotfactory.h"
#include "robot.h"
#include "robotsingleton.h"
#include "robotnormal.h"
#include "robotdebug.h"

RobotFactory::RobotFactory()
{
}

/**
 * @brief RobotFactory::Create Creates a new robot in either normal or debug
 * mode.
 * @param mode The run time mode of the robot (normal or debug)
 * @return Robot a new robot
 */
Robot* RobotFactory::Create(int mode) {
    if (mode == RobotFactory::RUN_MODE_DEBUG) {
        return new RobotDebug();
    } else {
        return new RobotNormal();
    }
}
