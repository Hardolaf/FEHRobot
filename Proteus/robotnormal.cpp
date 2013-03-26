#include "robotnormal.h"
#include "robot.h"
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHServo.h>

RobotNormal::RobotNormal()
{
    setup();
}

void RobotNormal::setup() {
    // Set up motors
    leftMotor = new FEHMotor( FEHMotor::Motor0 );
    rightMotor = new FEHMotor( FEHMotor::Motor1 );

    // Set up  left and right encoders
    leftEncoder = new FEHEncoder( FEHIO::P0_2 );
    float leftLowThreshold = 0.388;
    float leftHighThreshold = 1.547;
    leftEncoder->SetThresholds( leftLowThreshold, leftHighThreshold );

    rightEncoder = new FEHEncoder( FEHIO::P0_3);
    float rightLowThreshold = 0.388;
    float rightHighThreshold = 1.547;
    leftEncoder->SetThresholds( rightLowThreshold, rightHighThreshold );

    // Set up elevator servo
    elevator =  new FEHServo( FEHServo::Servo0 );
}

void RobotNormal::movementStraight(int speed, float distance) {
    int encoderCounts = Robot::ENCODER_COUNTS_PER_INCH * distance;
    leftEncoder->ResetCounts();
    rightEncoder->ResetCounts();

    leftMotor->SetPower(speed);
    rightMotor->SetPower(speed);

    // Encoder counts

    leftMotor->Stop();
    rightMotor->Stop();
}

void RobotNormal::movementLeft(int angle) {
    int encoderCounts = Robot::ENCODER_COUNTS_PER_DEGREE_TURN * angle;
    leftEncoder->ResetCounts();
    rightEncoder->ResetCounts();

    leftMotor->SetPower(-1 * Robot::MOVEMENT_MOTOR_TURN_SPEED);
    rightMotor->SetPower(Robot::MOVEMENT_MOTOR_TURN_SPEED);

    // Encoder counts

    leftMotor->Stop();
    rightMotor->Stop();
}

void RobotNormal::movementRight(int angle) {
    int encoderCounts = Robot::ENCODER_COUNTS_PER_DEGREE_TURN * angle;
    leftEncoder->ResetCounts();
    rightEncoder->ResetCounts();

    leftMotor->SetPower(Robot::MOVEMENT_MOTOR_TURN_SPEED);
    rightMotor->SetPower(-1 * Robot::MOVEMENT_MOTOR_TURN_SPEED);

    // Encoder counts

    leftMotor->Stop();
    rightMotor->Stop();
}
