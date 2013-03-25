#include "robot.h"
#ifndef ROBOTFACTORY_H
#define ROBOTFACTORY_H

class RobotFactory
{
public:
    static const int RUN_MODE_DEBUG = 0;
    static const int RUN_MODE_NORMAL = 1;
    static Robot* Create(int);
protected:
    RobotFactory();
};

#endif // ROBOTFACTORY_H
