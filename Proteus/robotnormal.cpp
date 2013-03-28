#include "robotnormal.h"
#include "robot.h"
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHServo.h>

/**
 * @brief RobotNormal::RobotNormal This function is a default constructor for
 * the robot normal class. It calls RobotNormal::setup() with the arugment true.
 * @see RobotNormal::setup()
 */
RobotNormal::RobotNormal()
{
    setup(true);
}

/**
 * @brief RobotNormal::setup Initalizes all objects on the robot.
 * @param calibrate True will calibrate the encoders and the servo motors.
 * @see RobotNormal::calibrate()
 */
void RobotNormal::setup(bool calibrate) {
    // Set up motors
    leftMotor = new FEHMotor( FEHMotor::Motor0 );
    rightMotor = new FEHMotor( FEHMotor::Motor1 );

    // Set up  left and right encoders
    leftEncoder = new FEHEncoder( FEHIO::P0_2 );
    rightEncoder = new FEHEncoder( FEHIO::P0_3);

    // Set up elevator servo
    elevator =  new FEHServo( FEHServo::Servo0 );

    // Set up light sensor (CDS cell)
    lightSensor = new AnalogInputPin( FEHIO::P1_7 );

    // Set up analog optosenors
    optosensorLeft = new AnalogInputPin( FEHIO::P1_6 );
    optosensorMiddle = new AnalogInputPin( FEHIO::P1_5 );
    optosensorRight = new AnalogInputPin( FEHIO::P1_4 );

    // Set up buttons
    buttons = new ButtonBoard( FEHIO::Bank3 );

    // Set up bump switches
    bumpSwitchFrontRight = new DigitalInputPin( FEHIO::P2_0 );
    bumpSwitchFrontLeft = new DigitalInputPin( FEHIO::P2_1 );
    bumpSwitchBackRight = new DigitalInputPin( FEHIO::P2_2 );
    bumpSwitchBackLeft = new DigitalInputPin( FEHIO::P2_3 );

    // Calibrate the robot if we need to
    if (calibrate) {
        RobotNormal::calibrate();
    }
}

/**
 * @brief RobotNormal::calibrate Calibrates encoders and the elevator servo.
 */
void RobotNormal::calibrate() {
    float leftLowThreshold = 3.050;
    float leftHighThreshold = 3.200;
    leftEncoder->SetThresholds( leftLowThreshold, leftHighThreshold );

    float rightLowThreshold = 3.050;
    float rightHighThreshold = 3.130;
    rightEncoder->SetThresholds( rightLowThreshold, rightHighThreshold );

    elevator->SetMin( 411 );
    elevator->SetMax( 3823 );
}

/**
 * @brief RobotNormal::movementMotorManualSet Sets the speeds of the tow motors
 * manually.
 * @param speedLeft [0, 127] where 0 is stopped, 127 is max speed.
 * @param speedRight [0, 127] where 0 is stopped, 127 is max speed.
 */
void RobotNormal::movementMotorManualSet(int speedLeft, int speedRight) {
    leftMotor->SetPower(speedLeft);
    leftMotor->SetPower(speedRight);
}

/**
 * @brief RobotNormal::movementStraight Moves the robot forwards or backwards
 * a given distance at a given speed. This is not exact motion due to error
 * inherent in shaft encoding.
 * @param speed [0, 127] where 0 is stopped, 127 is max speed.
 * @param distance (0, FLOAT_MAX] the distance the robot will travel.
 */
void RobotNormal::movementStraight(int speed, float distance) {
    // Declare number of encoder counts needed to travel
    int encoderCounts = Robot::ENCODER_COUNTS_PER_INCH * distance;

    // Reset encoder counters
    leftEncoder->ResetCounts();
    rightEncoder->ResetCounts();

    // Set the motor speed
    leftMotor->SetPower(speed);
    rightMotor->SetPower(speed);

    // Wait for the encoder to travel the specified distance
    while (leftEncoder->Counts() < encoderCounts
           && rightEncoder->Counts() < encoderCounts);

    // Stop the motors
    leftMotor->Stop();
    rightMotor->Stop();
}

/**
 * @brief RobotNormal::movementLeft The robot will turn a given angle left using
 * shaft encoding. Error does exist.
 * @param angle [0, INT_MAX] the angle the robot will turn
 */
void RobotNormal::movementLeft(int angle) {
    // Declare number of encoder counts needed to travel
    int encoderCounts = Robot::ENCODER_COUNTS_PER_DEGREE_TURN * angle;

    // Reset encoder counters
    leftEncoder->ResetCounts();
    rightEncoder->ResetCounts();

    // Set the motor speed
    leftMotor->SetPower(-1 * Robot::MOVEMENT_MOTOR_TURN_SPEED);
    rightMotor->SetPower(Robot::MOVEMENT_MOTOR_TURN_SPEED);

    // Wait for the encoder to travel the specified distance
    while (leftEncoder->Counts() < encoderCounts
           && rightEncoder->Counts() < encoderCounts);

    // Stop the motors
    leftMotor->Stop();
    rightMotor->Stop();
}

/**
 * @brief RobotNormal::movementRight The robot will turn a given angle left using
 * shaft encoding. Error does exist.
 * @param angle [0, INT_MAX] the angle the robot will turn
 */
void RobotNormal::movementRight(int angle) {
    // Declare number of encoder counts needed to travel
    int encoderCounts = Robot::ENCODER_COUNTS_PER_DEGREE_TURN * angle;

    // Reset encoder counters
    leftEncoder->ResetCounts();
    rightEncoder->ResetCounts();

    // Set the motor speed
    leftMotor->SetPower(Robot::MOVEMENT_MOTOR_TURN_SPEED);
    rightMotor->SetPower(-1 * Robot::MOVEMENT_MOTOR_TURN_SPEED);

    // Wait for the encoder to travel the specified distance
    while (leftEncoder->Counts() < encoderCounts
           && rightEncoder->Counts() < encoderCounts);

    // Stop the motors
    leftMotor->Stop();
    rightMotor->Stop();
}

bool RobotNormal::lightSensorSeeStart() {
    return lightSensor->Value() < LIGHT_SENSOR_MAX_LIGHT_VALUE;
}

bool RobotNormal::bumpSwitchFrontEitherPressed() {
    return bumpSwitchFrontLeftPressed() || bumpSwitchFrontRightPressed();
}

bool RobotNormal::bumpSwitchFrontLeftPressed() {
    return bumpSwitchFrontLeft->Value() == 0;
}

bool RobotNormal::bumpSwitchFrontRightPressed() {
    return bumpSwitchFrontRight == 0;
}

bool RobotNormal::bumpSwitchFrontBothPressed() {
    return bumpSwitchFrontLeftPressed() && bumpSwitchFrontRightPressed();
}

bool RobotNormal::bumpSwitchBackEitherPressed() {
    return bumpSwitchBackLeftPressed() || bumpSwitchBackRightPressed();
}

bool RobotNormal::bumpSwitchBackLeftPressed() {
    return bumpSwitchBackLeft->Value() == 0;
}

bool RobotNormal::bumpSwitchBackRightPressed() {
    return bumpSwitchBackRight->Value() ==0;
}

bool RobotNormal::bumpSwitchBackBothPressed() {
    return bumpSwitchBackLeftPressed() && bumpSwitchBackRightPressed();
}

bool RobotNormal::buttonRightPressed() {
    return buttons->RightPressed();
}

bool RobotNormal::buttonMiddlePressed() {
    return buttons->MiddlePressed();
}

bool RobotNormal::buttonLeftPressed() {
    return buttons->LeftPressed();
}

int RobotNormal::optosensorLeftSeesLine() {
    if (optosensorLeft->Value() > OPTOSENSOR_LEFT_HIGH_THRESHOLD) {
        return 1;
    } else if (optosensorLeft->Value() < OPTOSENSOR_LEFT_LOW_THRESHOLD) {
        return -1;
    } else {
        return 0;
    }
}

int RobotNormal::optosensorMiddleSeesLine() {
    if (optosensorMiddle->Value() > OPTOSENSOR_MIDDLE_HIGH_THRESHOLD) {
        return 1;
    } else if (optosensorMiddle->Value() < OPTOSENSOR_MIDDLE_LOW_THRESHOLD) {
        return -1;
    } else {
        return 0;
    }
}

int RobotNormal::optosensorRightSeesLine() {
    if (optosensorRight->Value() > OPTOSENSOR_RIGHT_HIGH_THRESHOLD) {
        return 1;
    } else if (optosensorRight->Value() < OPTOSENSOR_RIGHT_LOW_THRESHOLD) {
        return -1;
    } else {
        return 0;
    }
}
