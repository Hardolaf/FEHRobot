#include "robotdebug.h"
#include "robotnormal.h"
#include <FEHLCD.h>
#include <FEHUtility.h>

RobotDebug::RobotDebug()
{
    RobotNormal::setup(false);
}

RobotDebug::RobotDebug(bool calibrate)
{
    RobotNormal::setup(calibrate);
}

void RobotDebug::calibrateEncoders() {
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    float leftMin = 10.0, rightMin = 10.0;
    float leftMax = 0.0, rightMax = 0.0;

    while (!buttons->MiddlePressed()) {
        if (leftMin > leftEncoder->Value()) {
            leftMin = leftEncoder->Value();
        }
        if (leftMax < leftEncoder->Value()) {
            leftMax = leftEncoder->Value();
        }
        if (rightMin > rightEncoder->Value()) {
            rightMin = rightEncoder->Value();
        }
        if (rightMax < rightEncoder->Value()) {
            rightMax = rightEncoder->Value();
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
        LCD.Write(leftEncoder->Value());
        LCD.Write("   Right: ");
        LCD.WriteLine(rightEncoder->Value());
    }
}

void RobotDebug::calibrateOptosensors() {

}

void RobotDebug::calibrateLightSensor() {
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    float min = 4.0, max = 0.0;

    while (!buttons->MiddlePressed()) {
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

void RobotDebug::testMovementForward() {
    LCD.Clear( FEHLCD::Black );
    LCD.WriteLine("Test Forward Movement");

    this->calibrate();

    movementStraight(127, 6.0);
    Sleep(500);

    movementLeft(90);
    movementRight(90);

    LCD.WriteLine("");
    LCD.Write("Right Encoder: ");
    LCD.WriteLine(rightEncoder->Counts());
    LCD.WriteLine("");
    LCD.Write("Left Encoder: ");
    LCD.WriteLine(leftEncoder->Counts());
    LCD.WriteLine("");
    LCD.WriteLine("Press middle button to return to menu.");

    while (!buttons->MiddlePressed());
}
