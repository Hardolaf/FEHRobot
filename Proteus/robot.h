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
    static const float LIGHT_SENSOR_MAX_LIGHT_VALUE = 1.200;
};

#endif // ROBOT_H
