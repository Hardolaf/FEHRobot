#ifndef ROBOT_H
#define ROBOT_H

class Robot
{
public:
    virtual void movementStraight(int, float) = 0;
    virtual void movementLeft(int) = 0;
    virtual void movementRight(int) = 0;

    static const float ENCODER_COUNTS_PER_INCH = 3.316344860705540019742322139111;
    static const float ENCODER_COUNTS_PER_DEGREE_TURN = 0.23;

    static const int MOVEMENT_MOTOR_TURN_SPEED = 80;

    static const float LIGHT_SENSOR_MAX_LIGHT_VALUE = 2.000;

    static const float OPTOSENSOR_LEFT_LOW_THRESHOLD = 2.000;
    static const float OPTOSENSOR_LEFT_HIGH_THRESHOLD = 2.100;

    static const float OPTOSENSOR_MIDDLE_LOW_THRESHOLD = 2.000;
    static const float OPTOSENSOR_MIDDLE_HIGH_THRESHOLD = 2.100;

    static const float OPTOSENSOR_RIGHT_LOW_THRESHOLD = 2.000;
    static const float OPTOSENSOR_RIGHT_HIGH_THRESHOLD = 2.100;
};

#endif // ROBOT_H
