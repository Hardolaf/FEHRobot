#include "robotsingleton.h"

RobotSingleton* RobotSingleton::_instance = 0;

RobotSingleton* RobotSingleton::Instance() {
    if (_instance == 0) {
        _instance = new RobotSingleton;
    }
    return _instance;
}
