#ifndef ROBOT_H
#define ROBOT_H

class Robot
{
public:
    virtual void movementStraight(int, float) = 0;
    virtual void movementLeft(int, int) = 0;
    virtual void movementRight(int, int) = 0;
    static const float ENCODER_COUNTS_PER_INCH = 3.316344860705540019742322139111;
    static const float ENCODER_COUNTS_PER_DEGREE_TURN = 0.23;
    static const int MOVEMENT_MOTOR_TURN_SPEED = 50;
};

#endif // ROBOT_H
