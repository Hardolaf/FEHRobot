#ifndef ROBOTSINGLETON_H
#define ROBOTSINGLETON_H

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
