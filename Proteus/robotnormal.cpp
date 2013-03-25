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
    FEHMotor left_motor( FEHMotor::Motor0 );
    FEHMotor right_motor( FEHMotor::Motor1 );
    leftMotor = left_motor;
    right_motor = right_motor;
    elevatorServoMotor = FEHServo::Servo0;
}

void RobotNormal::movementStraight(int speed, float distance) {
    int encoderCounts = Robot::ENCODER_COUNTS_PER_INCH * distance;

    leftMotor.SetPower(speed);
    rightMotor.SetPower(speed);

    // Encoder counts

    leftMotor.Stop();
    rightMotor.Stop();
}

void RobotNormal::movementLeft(int angle) {
    int encoderCounts = Robot::ENCODER_COUNTS_PER_DEGREE_TURN * angle;

    leftMotor.SetPower(-1 * Robot::MOVEMENT_MOTOR_TURN_SPEED);
    rightMotor.SetPower(Robot::MOVEMENT_MOTOR_TURN_SPEED);

    // Encoder counts

    leftMotor.Stop();
    rightMotor.Stop();
}

void RobotNormal::movementRight(int angle) {
    int encoderCounts = Robot::ENCODER_COUNTS_PER_DEGREE_TURN * angle;

    leftMotor.SetPower(Robot::MOVEMENT_MOTOR_TURN_SPEED);
    rightMotor.SetPower(-1 * Robot::MOVEMENT_MOTOR_TURN_SPEED);

    // Encoder counts

    leftMotor.Stop();
    rightMotor.Stop();
}
