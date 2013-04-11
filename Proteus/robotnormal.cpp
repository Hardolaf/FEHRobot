#include "robotnormal.h"
#include "robot.h"
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHUtility.h>
#include <FEHLCD.h>

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

    // Set up encoders
    encoderLeft = new FEHEncoder( FEHIO::P2_6 );
    encoderRight = new FEHEncoder( FEHIO::P1_7);
    encoderSAM = new FEHEncoder ( FEHIO::P2_3 );

    // Set up servos
    servoElevator =  new FEHServo( FEHServo::Servo6 );
    servoArm = new FEHServo( FEHServo::Servo7 );

    // Set up light sensor (CDS cell)
    lightSensor = new AnalogInputPin( FEHIO::P1_1 );

    // Set up analog optosenors
    optosensorLeft = new AnalogInputPin( FEHIO::P1_3 );
    optosensorMiddle = new AnalogInputPin( FEHIO::P1_4 );
    optosensorRight = new AnalogInputPin( FEHIO::P1_6 );

    // Set up buttons
    buttons = new ButtonBoard( FEHIO::Bank3 );

    // Set up bump switches
    bumpSwitchFrontRight = new DigitalInputPin( FEHIO::P2_0 );
    bumpSwitchFrontLeft = new DigitalInputPin( FEHIO::P2_2 );
    bumpSwitchBackRight = new DigitalInputPin( FEHIO::P0_5 );
    bumpSwitchBackLeft = new DigitalInputPin( FEHIO::P0_7 );
    bumpSwitchBackMiddle = new DigitalInputPin( FEHIO::P2_7 );
    bumpSwitchArm = new DigitalInputPin( FEHIO::P2_4 );


    // Calibrate the robot if we need to
    if (calibrate) {
        RobotNormal::calibrate();
    }
}

/**
 * @brief RobotNormal::calibrate Calibrates encoders and the servos servo.
 */
void RobotNormal::calibrate() {
    float leftLowThreshold = 3.100;
    float leftHighThreshold = 3.200;
    encoderLeft->SetThresholds( leftLowThreshold, leftHighThreshold );

    float rightLowThreshold = 3.100;
    float rightHighThreshold = 3.200;
    encoderRight->SetThresholds( rightLowThreshold, rightHighThreshold );

//    float SAMLowThreshold = 0.050;
//    float SAMHighThreshold = 3.300;
//    encoderSAM->SetThresholds( SAMLowThreshold, SAMHighThreshold );

    servoElevator->SetMin( 1103 );
    servoElevator->SetMax( 2134 );

    servoArm->SetMin( 500 );
    servoArm->SetMax( 2414 );
}

/**
 * @brief RobotNormal::movementMotorManualSet Sets the speeds of the tow motors
 * manually.
 * @param speedLeft [-127, 127] where 0 is stopped, 127 is max speed.
 * @param speedRight [-127, 127] where 0 is stopped, 127 is max speed.
 */
void RobotNormal::movementMotorManualSet(int speedLeft, int speedRight) {
    motorLeft->SetPower(speedLeft);
    motorRight->SetPower(speedRight);
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
 * @brief RobotNormal::movementEncoderCountLeft Returns the current encoder
 * counts as measured by the left encoder.
 * @return Left encoder counts
 */
int RobotNormal::movementEncoderCountLeft() {
    return encoderLeft->Counts();
}

/**
 * @brief RobotNormal::movementEncoderCountRight Returns the current encoder
 * counts as measured by the right encoder.
 * @return Right encoder counts
 */
int RobotNormal::movementEncoderCountRight() {
    return encoderRight->Counts();
}

/**
 * @brief RobotNormal::movementEncoderCountReset Resets the encoder count as
 * stored by the right and left encoder objects.
 */
void RobotNormal::movementEncoderCountReset() {
    encoderLeft->ResetCounts();
    encoderRight->ResetCounts();
}

/**
 * @brief RobotNormal::movementFrontSquareToWall Squares the robot up
 * to the wall in front of it.
 */
void RobotNormal::movementFrontSquareToWall() {
    movementFrontSquareToWall(2000);
}

/**
 * @brief RobotNormal::movementFrontSquareToWall Squares the robot up
 * to the wall in front of it.
 * @param timeout Time between hitting the first button and stopping execution
 * of the forward motion.
 */
void RobotNormal::movementFrontSquareToWall(int timeout) {
    int last = -1;
    bool oneSensorHit = false;
    while(!bumpSwitchFrontBothPressed() && (!oneSensorHit || timeout > TimeNowMSec())) {
        if (bumpSwitchFrontEitherPressed()) {
            if (bumpSwitchFrontLeftPressed()) {
                if (last != 1) {
                    LCD.WriteLine("Left front pressed");
                    last = 1;
                    movementMotorManualSet(63, 127);
                    if (!oneSensorHit) {
                        timeout += TimeNowMSec();
                    }
                    oneSensorHit = true;
                }
            } else {
                if (last != 2) {
                    last = 2;
                    LCD.WriteLine("Right front pressed");
                    movementMotorManualSet(127, 63);
                    if (!oneSensorHit) {
                        timeout += TimeNowMSec();
                    }
                    oneSensorHit = true;
                }
            }
        } else {
            if (last != 0) {
                last = 0;
                movementMotorManualSet(127, 127);
                LCD.WriteLine("Neither front pressed");
            }
        }
    }

    Sleep(50);
    movementMotorManualSet(0, 0);
}

/**
 * @brief RobotNormal::movementBackSquaretoWall Squares the robot up
 * to the wall behind it.
 */
void RobotNormal::movementBackSquaretoWall() {
    int last = -1;
    while(!bumpSwitchBackBothPressed()) {
        if (bumpSwitchBackEitherPressed()) {
            if (bumpSwitchBackLeftPressed()) {
                if (last != 1) {
                    LCD.WriteLine("Left back pressed");
                    last = 1;
                    movementMotorManualSet(-60, -110);
                }
            } else {
                if (last != 2) {
                    last = 2;
                    LCD.WriteLine("Right back pressed");
                    movementMotorManualSet(-110, -60);
                }
            }
        } else {
            if (last != 0) {
                last = 0;
                movementMotorManualSet(-80, -80);
                LCD.WriteLine("Neither back pressed");
            }
        }
    }

    Sleep(50);
    movementMotorManualSet(-80, -80);
    Sleep(100);
    movementMotorManualSet(0, 0);
}

/**
 * @brief RobotNormal::motorSAMOpen Opens the SAM enclosure.
 */
void RobotNormal::motorSAMOpen() {
    encoderSAM->ResetCounts();

    // 90 degree rotation
    int encoderLimit = SAM_ENCODER_COUNTS_PER_DEGREE_TURN * 90;

    motorSAM->SetPower(-100);
    while(encoderSAM->Counts() < encoderLimit);
    motorSAM->SetPower(0);
}

/**
 * @brief RobotNormal::motorSAMClose Closes the SAM enclosure.
 */
void RobotNormal::motorSAMClose() {
    encoderSAM->ResetCounts();

    // 90 degree rotation
    int encoderLimit = SAM_ENCODER_COUNTS_PER_DEGREE_TURN * 90;

    motorSAM->SetPower(100);
    while(encoderSAM->Counts() < encoderLimit);
    motorSAM->SetPower(0);
}

/**
 * @brief Robot::motorSAMsetManualPower Sets the power of SAM motor manually
 * taking only an argument of a signed 8-bit integer which is the numerical
 * representation of the power setting.
 * @param power The power setting of the motor [-127, 127]
 */
void RobotNormal::motorSAMsetManualPower(int8 power) {
    motorSAM->SetPower(power);
}

/**
 * @brief servoElevatorSetAngle Sets the angle of the servo elevator. Includes
 * checks to constrain the angle within the domain of the function in order to
 * not exceed the operational range of the elevator.
 * @param angle The angle that the servo will bet set to [32, 162]
 */
void RobotNormal::servoElevatorSetAngle(int angle) {
    // Correct for out of bounds angles
    if (angle < 0)
        angle = 0;
    if (angle > 156)
        angle = 156;

    servoElevator->SetDegree(angle);
}

/**
 * @brief RobotNormal::servoElevatorHighest Sets the elevator to its highest
 * position.
 */
void RobotNormal::servoElevatorHighest() {
    servoElevatorSetAngle(180);
}

/**
 * @brief RobotNormal::servoElevatorLowest Sets the elevator to its lowest
 * position.
 */
void RobotNormal::servoElevatorLowest() {
    servoElevatorSetAngle(0);
}

/**
 * @brief servoArmSetAngle Sets the arm's servo to the specified angle within
 * the domain of the function (it corrects for invalid input) in order to not
 * exceed the operational range of the arm.
 * @param angle The angle that the servo will be set to [21, 124]
 */
void RobotNormal::servoArmSetAngle(int angle) {
    if (angle < 21)
        angle = 21;
    if (angle > 124)
        angle = 124;
    servoArm->SetDegree(angle);
}

/**
 * @brief RobotNormal::servoArmHighest Sets the arm to its furthest up position.
 */
void RobotNormal::servoArmHighest() {
    servoArmSetAngle(0);
}

/**
 * @brief RobotNormal::servoArmLowest Sets the arm to its lowest down position.
 */
void RobotNormal::servoArmLowest() {
    servoArmSetAngle(180);
}

/**
 * @brief RobotNormal::servoArmSetTask Sets the arm to the location at which is
 * needed in order to complete most tasks on the course.
 */
void RobotNormal::servoArmSetTask() {
    servoArmSetAngle(107);
}

/**
 * @brief RobotNormal::lightSensorSeeStart Checks to see if CDS cell "sees" a
 * light of any color.
 * @return True on sees light, false on does not see light
 */
bool RobotNormal::lightSensorSeeStart() {
    return (lightSensor->Value() < LIGHT_SENSOR_MAX_LIGHT_VALUE);
}

/**
 * @brief RobotNormal::bumpSwitchFrontEitherPressed Checks to see if either of
 * the two front bump switches are pressed at the time of the function call.
 * @return True for either pressed, false for neither pressed.
 */
bool RobotNormal::bumpSwitchFrontEitherPressed() {
    return (bumpSwitchFrontLeftPressed() || bumpSwitchFrontRightPressed());
}

/**
 * @brief RobotNormal::bumpSwitchFrontLeftPressed Checks to see if the left
 * front bump switch is pressed at the time of the function call.
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::bumpSwitchFrontLeftPressed() {
    return (bumpSwitchFrontLeft->Value() == 0);
}

/**
 * @brief RobotNormal::bumpSwitchFrontRightPressed Checks to see if the right
 * front bump switch is pressed at the time of the function.
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::bumpSwitchFrontRightPressed() {
    return (bumpSwitchFrontRight->Value() == 0);
}

/**
 * @brief RobotNormal::bumpSwitchFrontBothPressed Checks to see if both front
 * bump switches are pressed at the time of the function call.
 * @return True for both pressed, false for all other cases.
 */
bool RobotNormal::bumpSwitchFrontBothPressed() {
    return (bumpSwitchFrontLeftPressed() && bumpSwitchFrontRightPressed());
}

/**
 * @brief RobotNormal::bumpSwitchBackEitherPressed Checks to see if the either
 * of the rear bump switches are pressed at the time of the function call.
 * @return True for either pressed, false for neither pressed.
 */
bool RobotNormal::bumpSwitchBackEitherPressed() {
    return (bumpSwitchBackLeftPressed() || bumpSwitchBackRightPressed());
}

/**
 * @brief RobotNormal::bumpSwitchBackLeftPressed Checks to see if the rear left
 * bump switch is pressed at the time of the function call.
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::bumpSwitchBackLeftPressed() {
    return (bumpSwitchBackLeft->Value() == 0);
}

/**
 * @brief RobotNormal::bumpSwitchBackRightPressed Checks to see if the rear
 * right bump switch is pressed at the time of the function call.
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::bumpSwitchBackRightPressed() {
    return (bumpSwitchBackRight->Value() == 0);
}

/**
 * @brief RobotNormal::bumpSwitchBackBothPressed Checks to see if both of the
 * rear bump switches are pressed at the time of the function call.
 * @return True for both pressed, false for all other cases.
 */
bool RobotNormal::bumpSwitchBackBothPressed() {
    return (bumpSwitchBackLeftPressed() && bumpSwitchBackRightPressed());
}

/**
 * @brief RobotNormal::bumpSwitchBackMiddlePressed Checks to see if the middle
 * rear bump switch is pressed at the time of the function call.
 * @return True for pressed, false for other cases.
 */
bool RobotNormal::bumpSwitchBackMiddlePressed() {
    return (bumpSwitchBackMiddle->Value() == 0);
}

/**
 * @brief RobotNormal::bumpSwitchArmPressed Checks to see if the bump switch on
 * the robot's arm is pressed at the time of the function call.
 * @return True for pressed, false for other cases.
 */
bool RobotNormal::bumpSwitchArmPressed() {
    return (bumpSwitchArm->Value() == 0);
}

/**
 * @brief RobotNormal::buttonRightPressed Checks to see if the right button on
 * the button board is pressed.
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::buttonRightPressed() {
    return buttons->RightPressed();
}

/**
 * @brief RobotNormal::buttonMiddlePressed Checks to see if the middle button on
 * the button board is pressed.
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::buttonMiddlePressed() {
    return buttons->MiddlePressed();
}

/**
 * @brief RobotNormal::buttonLeftPressed Checks to see if the left button on the
 * button board is pressed.
 * @return True for pressed, false for not pressed.
 */
bool RobotNormal::buttonLeftPressed() {
    return buttons->LeftPressed();
}

/**
 * @brief RobotNormal::optosensorLeftSeesLine Checks to see if the robot can
 * currently see the line with the left optosensor.
 * @return 1 for sees line, -1 for does not see line, 0 for indeterminate.
 */
int RobotNormal::optosensorLeftSeesLine() {
    if (optosensorLeft->Value() > OPTOSENSOR_LEFT_HIGH_THRESHOLD) {
        return -1;
    } else if (optosensorLeft->Value() < OPTOSENSOR_LEFT_LOW_THRESHOLD) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief RobotNormal::optosensorMiddleSeesLine Checks to see if the robot can
 * currently see the line with the middle optosensor.
 * @return 1 for sees line, -1 for does not see line, 0 for indeterminate.
 */
int RobotNormal::optosensorMiddleSeesLine() {
    if (optosensorMiddle->Value() > OPTOSENSOR_MIDDLE_HIGH_THRESHOLD) {
        return -1;
    } else if (optosensorMiddle->Value() < OPTOSENSOR_MIDDLE_HIGH_THRESHOLD) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief RobotNormal::optosensorRightSeesLine Checks to see if the robot can
 * currently see the line with the right optosensor.
 * @return 1 for sees line, -1 for does not see line, 0 for indeterminate.
 */
int RobotNormal::optosensorRightSeesLine() {
    if (optosensorRight->Value() > OPTOSENSOR_RIGHT_HIGH_THRESHOLD) {
        return -1;
    } else if (optosensorRight->Value() < OPTOSENSOR_RIGHT_LOW_THRESHOLD) {
        return 1;
    } else {
        return 0;
    }
}
