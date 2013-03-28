#include "robotsingleton.h"

RobotSingleton* RobotSingleton::_instance = 0;

/**
 * @brief RobotSingleton::Instance
 * @return RobotSingleton
 */
RobotSingleton* RobotSingleton::Instance() {
    if (_instance == 0) {
        _instance = new RobotSingleton;
    }
    return _instance;
}
