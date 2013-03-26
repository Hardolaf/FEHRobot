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
    leftOptosenor = new AnalogInputPin( FEHIO::P1_6 );
    leftOptosenor = new AnalogInputPin( FEHIO::P1_5 );
    leftOptosenor = new AnalogInputPin( FEHIO::P1_4 );

    // Set up buttons
    buttons = new ButtonBoard( FEHIO::Bank3 );

    // Calibrate the robot if we need to
    if (calibrate) {
        RobotNormal::calibrate();
    }
}

void RobotNormal::calibrate() {
    float leftLowThreshold = 0.388;
    float leftHighThreshold = 1.547;
    leftEncoder->SetThresholds( leftLowThreshold, leftHighThreshold );

    float rightLowThreshold = 0.388;
    float rightHighThreshold = 1.547;
    rightEncoder->SetThresholds( rightLowThreshold, rightHighThreshold );

    elevator->SetMin( 411 );
    elevator->SetMax( 3823 );
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
    while (leftEncoder->Counts() > encoderCounts
           && rightEncoder->Counts() > encoderCounts);

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
    while (leftEncoder->Counts() > encoderCounts
           && rightEncoder->Counts() > encoderCounts);

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
    while (leftEncoder->Counts() > encoderCounts
           && rightEncoder->Counts() > encoderCounts);

    // Stop the motors
    leftMotor->Stop();
    rightMotor->Stop();
}
