#ifndef ROBOTSINGLETON_H
#define ROBOTSINGLETON_H

/**
 * Defines a robot of which only one can exist at any given point in time.
 */
class RobotSingleton
{
public:
    static RobotSingleton* Instance();
protected:
    RobotSingleton();
private:
    static RobotSingleton* _instance;
};

#endif // ROBOTSINGLETON_H
