#include "robotnormal.h"
#include "robot.h"
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHServo.h>

RobotNormal::RobotNormal()
{
    setup(true);
}

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

void RobotNormal::movementMotorManualSet(int speedLeft, int speedRight) {
    leftMotor->SetPower(speedLeft);
    leftMotor->SetPower(speedRight);
}

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
