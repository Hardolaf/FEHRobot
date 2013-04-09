#ifndef ROBOT_H
#define ROBOT_H

/**
 * Robot represents the base class of all other derived robots. It defines all
 * run time constants that are not passed directly to objects. It defines a
 * limited subset of functions that must be implemented by all robots.
 */
class Robot
{
public:
    virtual void movementStraight(int, float) = 0;
    virtual void movementLeft(int) = 0;
    virtual void movementRight(int) = 0;

    /**
     * Encoder counts per linear inch of motion.
     */
    static const float ENCODER_COUNTS_PER_INCH = 3.4545454545454545454545454545455;
    /**
     * Encoder counts per degree of turning in both drive wheels..
     */
    static const float ENCODER_COUNTS_PER_DEGREE_TURN = 0.22222222222222222222222222222222;

    /**
     * Encoder counts per degree of turning in the SAM motor (Faulhaber).
     */
    static const float SAM_ENCODER_COUNTS_PER_DEGREE_TURN = 0.80740740740740740740740740740741;

    /**
     * Drive motor speed out of 128 for turning.
     */
    static const int MOVEMENT_MOTOR_TURN_SPEED = 70;

    /**
     * Voltage reading from light sensor (CDS Cell) which signifies the
     * difference between the start light being on or off.
     */
    static const float LIGHT_SENSOR_MAX_LIGHT_VALUE = 2.000;

    /**
     * Low threshold of the left line following optosensor.
     */
    static const float OPTOSENSOR_LEFT_LOW_THRESHOLD = 3.060;
    /**
     * High threshold of the left line following optosensor.
     */
    static const float OPTOSENSOR_LEFT_HIGH_THRESHOLD = 3.150;

    /**
     * Low threshold of the middle line following optosensor.
     */
    static const float OPTOSENSOR_MIDDLE_LOW_THRESHOLD = 3.100;
    /**
     * High threshold of the middle line following optosensor.
     */
    static const float OPTOSENSOR_MIDDLE_HIGH_THRESHOLD = 3.150;

    /**
     * Low threshold of the right line following optosensor.
     */
    static const float OPTOSENSOR_RIGHT_LOW_THRESHOLD = 3.100;
    /**
     * High threshold of the right line following optosensor.
     */
    static const float OPTOSENSOR_RIGHT_HIGH_THRESHOLD = 3.180;
};

#endif // ROBOT_H
