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
    motorLeft = new FEHMotor( FEHMotor::Motor0 );
    motorRight = new FEHMotor( FEHMotor::Motor1 );
    motorSAM = new FEHMotor( FEHMotor::Motor2);

    // Set up  left and right encoders
    encoderLeft = new FEHEncoder( FEHIO::P0_5 );
    encoderRight = new FEHEncoder( FEHIO::P0_7);

    // Set up elevator servo
    elevator =  new FEHServo( FEHServo::Servo0 ); // not set

    // Set up light sensor (CDS cell)
    lightSensor = new AnalogInputPin( FEHIO::P0_1 );

    // Set up analog optosenors
    optosensorLeft = new AnalogInputPin( FEHIO::P1_6 ); // not set
    optosensorMiddle = new AnalogInputPin( FEHIO::P1_5 ); // not set
    optosensorRight = new AnalogInputPin( FEHIO::P1_4 ); // not set

    // Set up buttons
    buttons = new ButtonBoard( FEHIO::Bank3 );

    // Set up bump switches
    bumpSwitchFrontRight = new DigitalInputPin( FEHIO::P1_2 );
    bumpSwitchFrontLeft = new DigitalInputPin( FEHIO::P1_4 );
    bumpSwitchBackRight = new DigitalInputPin( FEHIO::P1_6 ); // not set
    bumpSwitchBackLeft = new DigitalInputPin( FEHIO::P2_0 ); // not set

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
    encoderLeft->SetThresholds( leftLowThreshold, leftHighThreshold );

    float rightLowThreshold = 3.050;
    float rightHighThreshold = 3.130;
    encoderRight->SetThresholds( rightLowThreshold, rightHighThreshold );

    elevator->SetMin( 411 );
    elevator->SetMax( 3823 );
}

/**
 * @brief RobotNormal::movementMotorManualSet Sets the speeds of the tow motors
 * manually.
 * @param speedLeft [-127, 127] where 0 is stopped, 127 is max speed.
 * @param speedRight [-127, 127] where 0 is stopped, 127 is max speed.
 */
void RobotNormal::movementMotorManualSet(int speedLeft, int speedRight) {
    motorLeft->SetPower(speedLeft);
    motorLeft->SetPower(speedRight);
}

/**
 * @brief RobotNormal::movementStraight Moves the robot forwards or backwards
 * a given distance at a given speed. This is not exact motion due to error
 * inherent in shaft encoding.
 * @param speed [-127, 127] where 0 is stopped, 127 is max speed.
 * @param distance (0, FLOAT_MAX] the distance the robot will travel.
 */
void RobotNormal::movementStraight(int speed, float distance) {
    // Declare number of encoder counts needed to travel
    int encoderCounts = Robot::ENCODER_COUNTS_PER_INCH * distance;

    // Reset encoder counters
    encoderLeft->ResetCounts();
    encoderRight->ResetCounts();

    // Set the motor speed
    motorLeft->SetPower(speed);
    motorRight->SetPower(speed);

    // Wait for the encoder to travel the specified distance
    while (encoderLeft->Counts() < encoderCounts
           && encoderRight->Counts() < encoderCounts);

    // Stop the motors
    motorLeft->Stop();
    motorRight->Stop();
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
    encoderLeft->ResetCounts();
    encoderRight->ResetCounts();

    // Set the motor speed
    motorLeft->SetPower(-1 * Robot::MOVEMENT_MOTOR_TURN_SPEED);
    motorRight->SetPower(Robot::MOVEMENT_MOTOR_TURN_SPEED);

    // Wait for the encoder to travel the specified distance
    while (encoderLeft->Counts() < encoderCounts
           && encoderRight->Counts() < encoderCounts);

    // Stop the motors
    motorLeft->Stop();
    motorRight->Stop();
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
    movementEncoderCountReset();

    // Set the motor speed
    motorLeft->SetPower(Robot::MOVEMENT_MOTOR_TURN_SPEED);
    motorRight->SetPower(-1 * Robot::MOVEMENT_MOTOR_TURN_SPEED);

    // Wait for the encoder to travel the specified distance
    while (encoderLeft->Counts() < encoderCounts
           && encoderRight->Counts() < encoderCounts);

    // Stop the motors
    motorLeft->Stop();
    motorRight->Stop();
}

/**
 * @brief RobotNormal::movementEncoderCountLeft
 * @return Left encoder counts
 */
int RobotNormal::movementEncoderCountLeft() {
    return encoderLeft->Counts();
}

/**
 * @brief RobotNormal::movementEncoderCountRight
 * @return Right encoder counts
 */
int RobotNormal::movementEncoderCountRight() {
    return encoderRight->Counts();
}

/**
 * @brief RobotNormal::movementEncoderCountReset
 */
void RobotNormal::movementEncoderCountReset() {
    encoderLeft->ResetCounts();
    encoderRight->ResetCounts();
}

/**
 * @brief RobotNormal::motorSAMOpen Opens the SAM enclosure
 */
void RobotNormal::motorSAMOpen() {
    motorSAM->SetPower(126/2);
    Sleep(0.5);
}

/**
 * @brief RobotNormal::motorSAMClose Closes the SAM enclosure
 */
void RobotNormal::motorSAMClose() {
    motorSAM->SetPower(-126/2);
    Sleep(0.5)
}

/**
 * @brief RobotNormal::lightSensorSeeStart Checks to see if CDS cell "sees" a
 * light of any color.
 * @return True on sees light, false on does not see light
 */
bool RobotNormal::lightSensorSeeStart() {
    return lightSensor->Value() < LIGHT_SENSOR_MAX_LIGHT_VALUE;
}

/**
 * @brief RobotNormal::bumpSwitchFrontEitherPressed
 * @return True for either pressed, false for neither pressed.
 */
bool RobotNormal::bumpSwitchFrontEitherPressed() {
    return bumpSwitchFrontLeftPressed() || bumpSwitchFrontRightPressed();
}

/**
 * @brief RobotNormal::bumpSwitchFrontLeftPressed
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::bumpSwitchFrontLeftPressed() {
    return bumpSwitchFrontLeft->Value() == 0;
}

/**
 * @brief RobotNormal::bumpSwitchFrontRightPressed
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::bumpSwitchFrontRightPressed() {
    return bumpSwitchFrontRight == 0;
}

/**
 * @brief RobotNormal::bumpSwitchFrontBothPressed
 * @return True for both pressed, false for all other cases.
 */
bool RobotNormal::bumpSwitchFrontBothPressed() {
    return bumpSwitchFrontLeftPressed() && bumpSwitchFrontRightPressed();
}

/**
 * @brief RobotNormal::bumpSwitchBackEitherPressed
 * @return True for either pressed, false for neither pressed.
 */
bool RobotNormal::bumpSwitchBackEitherPressed() {
    return bumpSwitchBackLeftPressed() || bumpSwitchBackRightPressed();
}

/**
 * @brief RobotNormal::bumpSwitchBackLeftPressed
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::bumpSwitchBackLeftPressed() {
    return bumpSwitchBackLeft->Value() == 0;
}

/**
 * @brief RobotNormal::bumpSwitchBackRightPressed
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::bumpSwitchBackRightPressed() {
    return bumpSwitchBackRight->Value() == 0;
}

/**
 * @brief RobotNormal::bumpSwitchBackBothPressed
 * @return True for both pressed, false for all other cases.
 */
bool RobotNormal::bumpSwitchBackBothPressed() {
    return bumpSwitchBackLeftPressed() && bumpSwitchBackRightPressed();
}

/**
 * @brief RobotNormal::buttonRightPressed
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::buttonRightPressed() {
    return buttons->RightPressed();
}

/**
 * @brief RobotNormal::buttonMiddlePressed
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::buttonMiddlePressed() {
    return buttons->MiddlePressed();
}

/**
 * @brief RobotNormal::buttonLeftPressed
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::buttonLeftPressed() {
    return buttons->LeftPressed();
}

/**
 * @brief RobotNormal::optosensorLeftSeesLine
 * @return 1 for sees line, -1 for does not see line, 0 for indeterminate.
 */
int RobotNormal::optosensorLeftSeesLine() {
    if (optosensorLeft->Value() > OPTOSENSOR_LEFT_HIGH_THRESHOLD) {
        return 1;
    } else if (optosensorLeft->Value() < OPTOSENSOR_LEFT_LOW_THRESHOLD) {
        return -1;
    } else {
        return 0;
    }
}

/**
 * @brief RobotNormal::optosensorMiddleSeesLine
 * @return 1 for sees line, -1 for does not see line, 0 for indeterminate.
 */
int RobotNormal::optosensorMiddleSeesLine() {
    if (optosensorMiddle->Value() > OPTOSENSOR_MIDDLE_HIGH_THRESHOLD) {
        return 1;
    } else if (optosensorMiddle->Value() < OPTOSENSOR_MIDDLE_LOW_THRESHOLD) {
        return -1;
    } else {
        return 0;
    }
}

/**
 * @brief RobotNormal::optosensorRightSeesLine
 * @return 1 for sees line, -1 for does not see line, 0 for indeterminate.
 */
int RobotNormal::optosensorRightSeesLine() {
    if (optosensorRight->Value() > OPTOSENSOR_RIGHT_HIGH_THRESHOLD) {
        return 1;
    } else if (optosensorRight->Value() < OPTOSENSOR_RIGHT_LOW_THRESHOLD) {
        return -1;
    } else {
        return 0;
    }
}
