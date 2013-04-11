#include "robotdebug.h"
#include "robotnormal.h"
#include <FEHLCD.h>
#include <FEHUtility.h>

/**
 * @brief RobotDebug::RobotDebug Sets up the new robot for running without
 * calibrating.
 * @see RobotNormal::setup()
 */
RobotDebug::RobotDebug()
{
    RobotNormal::setup(false);
}

/**
 * @brief RobotDebug::RobotDebug Sets up the new robot for running with or
 * withour calibration.
 * @param calibrate Passed to RobotNormal::setup()
 * @see RobotNormal::setup()
 */
RobotDebug::RobotDebug(bool calibrate)
{
    RobotNormal::setup(calibrate);
}

/**
 * @brief RobotDebug::calibrateEncoders Helps the user determine calibration
 * values for the shaft encoders by outputting their current values, their mins,
 * and their maxes during the course of the function.
 */
void RobotDebug::calibrateEncoders() {
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    float leftMin = 10.0, rightMin = 10.0;
    float leftMax = 0.0, rightMax = 0.0;

    while (!buttonMiddlePressed()) {
        if (leftMin > encoderLeft->Value()) {
            leftMin = encoderLeft->Value();
        }
        if (leftMax < encoderLeft->Value()) {
            leftMax = encoderLeft->Value();
        }
        if (rightMin > encoderRight->Value()) {
            rightMin = encoderRight->Value();
        }
        if (rightMax < encoderRight->Value()) {
            rightMax = encoderRight->Value();
        }

        // Print
        LCD.Clear( FEHLCD::Black );
        LCD.WriteLine("Encoder Calibration");
        LCD.WriteLine("  Press middle button to stop.");
        LCD.WriteLine("");
        LCD.WriteLine("Right:");
        LCD.Write("Min: ");
        LCD.Write(rightMin);
        LCD.Write("   Max: ");
        LCD.WriteLine(rightMax);
        LCD.WriteLine("Left:");
        LCD.Write("Min: ");
        LCD.Write(leftMin);
        LCD.Write("   Max: ");
        LCD.WriteLine(leftMax);
        LCD.WriteLine("");
        LCD.WriteLine("Current:");
        LCD.Write("Left: ");
        LCD.Write(encoderLeft->Value());
        LCD.Write("   Right: ");
        LCD.WriteLine(encoderRight->Value());
    }
}

/**
 * @brief RobotDebug::calibrateOptosensors Helps the user determine calibration
 * values for the line following optosensors by outputting their current values,
 * their mins, and their maxes during the course of the function.
 */
void RobotDebug::calibrateOptosensors() {
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    float mMin = 4.0, mMax = 0.0;
    float rMin = 4.0, rMax = 0.0;
    float lMin = 4.0, lMax = 0.0;

    while (!buttonMiddlePressed()) {
        if (optosensorMiddle->Value() > mMax) {
            mMax = optosensorMiddle->Value();
        }
        if (optosensorMiddle->Value() < mMin) {
            mMin = optosensorMiddle->Value();
        }
        if (optosensorRight->Value() > rMax) {
            rMax = optosensorRight->Value();
        }
        if (optosensorRight->Value() < rMin) {
            rMin = optosensorRight->Value();
        }
        if (optosensorLeft->Value() > lMax) {
            lMax = optosensorLeft->Value();
        }
        if (optosensorLeft->Value() < lMin) {
            lMin = optosensorLeft->Value();
        }

        // Print
        LCD.Clear( FEHLCD::Black );
        LCD.WriteLine("Optosensor Calibration");
        LCD.WriteLine("  Press middle button to stop.");
        LCD.WriteLine("");
        LCD.Write("M Min: ");
        LCD.Write(mMin);
        LCD.Write("   M Max: ");
        LCD.WriteLine(mMax);
        LCD.Write("L Min: ");
        LCD.Write(lMin);
        LCD.Write("   L Max: ");
        LCD.WriteLine(lMax);
        LCD.Write("R Min: ");
        LCD.Write(rMin);
        LCD.Write("   R Max: ");
        LCD.WriteLine(rMax);
        LCD.WriteLine("");
        LCD.WriteLine("Current: ");
        LCD.Write("M: ");
        LCD.WriteLine(optosensorMiddle->Value());
        LCD.Write("R: ");
        LCD.WriteLine(optosensorRight->Value());
        LCD.Write("L: ");
        LCD.WriteLine(optosensorLeft->Value());
        Sleep(50);
    }
}

/**
 * @brief RobotDebug::calibrateLightSensor Helps the user determine calibration
 * values for the light sensor (CDS cell) by outputting its current value, its
 * min, and its max during the course of the function.
 */
void RobotDebug::calibrateLightSensor() {
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    float min = 4.0, max = 0.0;

    while (!buttonMiddlePressed()) {
        if (lightSensor->Value() > max) {
            max = lightSensor->Value();
        }
        if (lightSensor->Value() < min) {
            min = lightSensor->Value();
        }

        // Print
        LCD.Clear( FEHLCD::Black );
        LCD.WriteLine("Light Sensor Calibration");
        LCD.WriteLine("  Press middle button to stop.");
        LCD.WriteLine("");
        LCD.Write("Min: ");
        LCD.Write(min);
        LCD.Write("   Max: ");
        LCD.WriteLine(max);
        LCD.WriteLine("");
        LCD.Write("Current: ");
        LCD.WriteLine(lightSensor->Value());
        Sleep(50);
    }
}

/**
 * @brief RobotDebug::testMovementForward Tests to see if forward motion is
 * working with shaft encoding. It moves the robot forward ~6.0 inches and then
 * outputs the number of encoder counts for each encoder 500 milliseconds after
 * the motors have been killed. This function is best used in conjunction with a
 * distance measuring tool.
 * @see RobotNormal::movementStraight()
 */
void RobotDebug::testMovementForward() {
    LCD.Clear( FEHLCD::Black );
    LCD.WriteLine("Test Forward Movement");

    this->calibrate();

    movementStraight(63, 6.0);
    //movementLeft(90);
    Sleep(500);

    LCD.WriteLine("");
    LCD.Write("Right Encoder: ");
    LCD.WriteLine(encoderRight->Counts());
    LCD.WriteLine("");
    LCD.Write("Left Encoder: ");
    LCD.WriteLine(encoderLeft->Counts());
    LCD.WriteLine("");
    LCD.WriteLine("Press middle button to return to menu.");

    while (!buttonMiddlePressed());
}

/**
 * @brief RobotDebug::testMotorSAM Tests the SAM motor ot enusre that it is
 * infact powered properly.
 * @see RobotNormal::motorSAMsetManualPower()
 */
void RobotDebug::testMotorSAM() {
    LCD.Clear( FEHLCD::Black );
    Sleep(500);
    LCD.WriteLine("Test SAM motor");
    LCD.WriteLine("  Press left button to power");
    LCD.WriteLine("  Press middle button to return to menu");

    while(!buttonMiddlePressed()) {
        if (buttonLeftPressed()) {
            motorSAMOpen();
            Sleep(200);
            LCD.Write("Encoder Counts: ");
            LCD.WriteLine(encoderSAM->Counts());
            Sleep(500);
        }
        if (buttonRightPressed()) {
            motorSAMClose();
            Sleep(200);
            LCD.Write("Encoder Counts: ");
            LCD.WriteLine(encoderSAM->Counts());
            Sleep(500);
        }
    }
    motorSAMsetManualPower(0);
}

/**
 * @brief RobotDebug::calibrateServoElevator Calibrates the elevator servo using
 * the built in calibration function.
 */
void RobotDebug::calibrateServoElevator() {
    LCD.Clear( FEHLCD::Black );

    servoElevator->Calibrate();
    Sleep(1000);

    LCD.WriteLine("");
    LCD.WriteLine("Press middle button to return to menu.");
    while (!buttonMiddlePressed());
}

/**
 * @brief RobotDebug::calibrateServoArm Calibrates the arm servo using the built
 * in calibration function.
 */
void RobotDebug::calibrateServoArm() {
    LCD.Clear( FEHLCD::Black );

    servoArm->Calibrate();
    Sleep(1000);

    LCD.WriteLine("");
    LCD.WriteLine("Press middle button to return to menu.");
    while (!buttonMiddlePressed());
}

/**
 * @brief RobotDebug::testServoElevator Tests the servo elevator by moving the
 * elevator up or down depending on which button is pressed by no more than one
 * degree changes in the servo motor every 50 milliseconds.
 */
void RobotDebug::testServoElevator() {
    this->calibrate();
    LCD.Clear( FEHLCD::Black );
    LCD.WriteLine("Test Elevator Servo");
    LCD.WriteLine("Press middle button to return to menu.");
    LCD.WriteLine("Press left button to decrease angle.");
    LCD.WriteLine("Press right button to increase angle.");

    int angle = 0;
    while (!buttonMiddlePressed()) {
        if (buttonLeftPressed()) {
            angle -=1;
        } else if (buttonRightPressed()) {
            angle +=1;
        }

        if (angle < 0) {
            angle = 0;
        } else if (angle > 180) {
            angle = 180;
        }

        servoElevator->SetDegree(angle);
        LCD.WriteLine(angle);
        Sleep(50);
    }
}

/**
 * @brief RobotDebug::testServoArm Tests the servo motor by moving the arm up or
 * down depending on which button is pressed by no more than one degree changes
 * in the servo motor every 50 milliseconds.
 */
void RobotDebug::testServoArm() {
    this->calibrate();
    LCD.Clear( FEHLCD::Black );
    LCD.WriteLine("Test Arm Servo");
    LCD.WriteLine("Press middle button to return to menu.");
    LCD.WriteLine("Press left button to decrease angle.");
    LCD.WriteLine("Press right button to increase angle.");

    int angle = 0;
    while (!buttonMiddlePressed()) {
        if (buttonLeftPressed()) {
            angle -=1;
        } else if (buttonRightPressed()) {
            angle +=1;
        }

        if (angle < 0) {
            angle = 0;
        } else if (angle > 180) {
            angle = 180;
        }

        servoArmSetAngle(angle);
        LCD.WriteLine(angle);
        Sleep(50);
    }
}

/**
 * @brief RobotDebug::testServoElevatorSetAngle This function moves the elevator
 * between its highest and lowest position depending on which button is pressed.
 * This is used to check to see if the correct minimum and correct maximum
 * angles of operation are defined.
 */
void RobotDebug::testServoElevatorSetAngle() {
    this->calibrate();
    LCD.Clear( FEHLCD::Black );
    LCD.WriteLine("Test Elevator Servo");
    LCD.WriteLine("Press middle button to return to menu.");
    LCD.WriteLine("Press left button to set to lowest position.");
    LCD.WriteLine("Press right button to set to highest position.");

    while (!buttonMiddlePressed()) {
        if (buttonRightPressed()) {
            servoElevatorHighest();
            Sleep(300);
        } else if (buttonLeftPressed()) {
            servoElevatorLowest();
            Sleep(300);
        }
    }
}
