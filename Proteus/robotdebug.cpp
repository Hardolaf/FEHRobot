#include "robotdebug.h"
#include "robotnormal.h"
#include <FEHLCD.h>

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

    int leftMin = 65535, rightMin = 65535;
    int leftMax = 0, rightMax = 0;

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
        if (rightMax > rightEncoder->Value()) {
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

}
