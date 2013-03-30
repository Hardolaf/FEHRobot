#include "robot.h"
#include "robotdebug.h"
#include "robotnormal.h"
#include "robotfactory.h"
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>

void debug_menu();
void performance_test_5(RobotNormal robot);

int main(void)
{
    ButtonBoard buttons( FEHIO::Bank3 );
    
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    LCD.WriteLine("Use the buttons to select the run mode.");
    LCD.WriteLine("Left: Normal");
    LCD.WriteLine("Right: Debug");



    int runTimeMode = -1;

    while( true )
    {
        if( buttons.LeftPressed() ) {
            runTimeMode = RobotFactory::RUN_MODE_NORMAL;
            break;
        } else if ( buttons.RightPressed()) {
            runTimeMode = RobotFactory::RUN_MODE_DEBUG;
            break;
        }
    }

    if (runTimeMode == RobotFactory::RUN_MODE_NORMAL) {
        RobotNormal robot;

        performance_test_5(robot);
    } else if (runTimeMode == RobotFactory::RUN_MODE_DEBUG) {
        RobotDebug robot;

        debug_menu();

        while ( true ) {
            if (buttons.LeftPressed()) {
                robot.calibrateEncoders();
                debug_menu();
            } else if (buttons.MiddlePressed()) {
                robot.testMovementForward();
                debug_menu();
            } else if (buttons.RightPressed()) {
                robot.calibrateLightSensor();
                debug_menu();
            }

            Sleep( 100 );
        }
    }

    return 0;
}

/**
 * @brief debug_menu Outputs the debug option menu after clearing the LCD.
 */
void debug_menu() {
    LCD.Clear( FEHLCD::Black );
    LCD.WriteLine("DEBUG MODE");
    LCD.WriteLine("Left: Calibrate Encoders");
    LCD.WriteLine("Middle: Test Forward Movement");
    LCD.WriteLine("Right: Calibrate Light Sensor");
    Sleep( 300 );
}

/**
 * @brief startup_sequence
 * @param robot A RobotNormal with RobotNormal::setup() and
 * RobotNormal::calibrate()run
 */
void startup_sequence(RobotNormal robot) {
    // Ask the user to press the middle button to prime it for waiting for the
    // light.
    LCD.Clear( FEHLCD::Scarlet );
    LCD.SetFontColor( FEHLCD::Gray );
    LCD.WriteLine("NORMAL MODE");
    LCD.WriteLine("");
    LCD.WriteLine("Press middle button to robot start.");
    Sleep(100);

    // Wait for the middle button to be pressed.
    while(!robot.buttonMiddlePressed());

    LCD.WriteLine("Ready in 1 second.");
    Sleep(1000);
    LCD.Clear( FEHLCD::Scarlet );
    LCD.WriteLine("Waiting for light.");

    // Wait for line
    while(!robot.lightSensorSeeStart());
    LCD.WriteLine("Run started.");
}

/**
 * @brief performance_test_5
 * @param robot A RobotNormal with RobotNormal::setup() and
 * RobotNormal::calibrate()run
 */
void performance_test_5(RobotNormal robot) {
    startup_sequence(robot);

    // Go up the stairs
    LCD.WriteLine("Forward 30 inches");
    robot.movementStraight(127, 30.0);
    Sleep(100);

    // Turn 45 degress towards wall
    LCD.WriteLine("Left 45 degrees");
    robot.movementLeft(45);
    Sleep(100);

    // Square up against the wall (turn into a function)
    robot.movementFrontSquareToWall();

    // Turn 90 towards pryramid (front facing)
    LCD.WriteLine("Back up 0.25 inches");
    robot.movementStraight(-63, 0.30);
    Sleep(50);
    LCD.WriteLine("Turning right");
    robot.movementRight(80);
    Sleep(100);

    // Charge the stone like a madman
    bool hitStone = false;
    int charges = 0;
    bool hfr = false;
    while(!hitStone || charges < 2) {
        LCD.WriteLine("Charge STONE");
        double end_time = TimeNow() + 2.0;

        // Move straight forward for 2.0 seconds
        robot.movementMotorManualSet(80, 80);
        while (end_time > TimeNow()) {
            LCD.WriteLine(robot.bumpSwitchBackRightPressed());
            if (robot.bumpSwitchBackRightPressed()) {
                hitStone = true;
                LCD.WriteLine("Hit STONE");
            }
            if (robot.bumpSwitchFrontRightPressed()) {
                hfr = true;
            }
        }
        Sleep(100);
        LCD.WriteLine("Move backwards");

        // Move it backwards
        if (hfr || hitStone) {
            robot.movementStraight(-100, 4.0);
        } else {
            robot.movementMotorManualSet(-80, -100);
            Sleep(250);
            robot.movementMotorManualSet(0, 0);
        }
        Sleep(100);
        charges++;
    }

    // Move backwards for ___ inches or _ seconds
    LCD.WriteLine("Performing SAM drop");
    robot.movementMotorManualSet(-80, -80);
    while(!robot.bumpSwitchBackLeftPressed());
    Sleep(100);
    robot.movementMotorManualSet(0, 0);
    Sleep(100);

    // Drop SAM in the hole.
    //robot.motorSAMOpen();
    //Sleep(100);

    // Close SAM enclosure.
    //robot.motorSAMClose();
    //Sleep(100);

    // Forward 10.0 inches
    LCD.WriteLine("Go forward");
    robot.movementStraight(80, 10.0);
    Sleep(100);

    // Turn right 135
    LCD.WriteLine("Turn right 135");
    robot.movementRight(120);
    Sleep(100);

    // Go down to basecamp
    LCD.WriteLine("Returning to basecamp");
    robot.movementFrontSquareToWall();

    // End run
    robot.movementMotorManualSet(0, 0);
    LCD.WriteLine("Run complete");
}
