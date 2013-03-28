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
    LCD.WriteLine("Waiting for line.");

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
    robot.movementStraight(127, 22.0);
    Sleep(100);

    // Move until we see the line
    robot.movementMotorManualSet(126/2, 126/2);
    while(robot.optosensorMiddleSeesLine() != 1);
    robot.movementMotorManualSet(0, 0);
    Sleep(100);

    // Turn 45 degress towards wall
    robot.movementLeft(45);
    Sleep(100);

    // Square up against the wall (turn into a function)
    int last = -1;
    while(!robot.bumpSwitchFrontBothPressed()) {
        if (robot.bumpSwitchFrontEitherPressed()) {
            if (robot.bumpSwitchFrontLeftPressed()) {
                if (last != 1) {
                    last = 1;
                    robot.movementMotorManualSet(63, 127);
                }
            } else {
                if (last != 2) {
                    last = 2;
                    robot.movementMotorManualSet(127, 63);
                }
            }
        } else {
            if (last != 0) {
                last = 0;
                robot.movementMotorManualSet(63, 63);
            }
        }
    }
    Sleep(50);
    robot.movementMotorManualSet(0, 0);

    // Turn 90 towards pryramid (front facing)
    robot.movementStraight(-63, 0.5);
    Sleep(50);
    robot.movementRight(90);
    Sleep(100);

    // Move straight forward for 11.0? inches or 5.0? seconds
    for (int i = 0; i < 2; i++) {
        float end_time = TimeNow() + 5.0;
        int encoder_count_limit = 11.0 * Robot::ENCODER_COUNTS_PER_INCH;

        // Move the robot forward
        robot.movementEncoderCountReset();
        robot.movementStraight(80, 80);
        while (end_time > TimeNow()
               || encoder_count_limit > robot.movementEncoderCountLeft()
               || encoder_count_limit > robot.movementEncoderCountRight());
        Sleep(100);

        // Move it backwards
        robot.movementEncoderCountReset();
        end_time = TimeNow() + 5.0;
        robot.movementStraight(-80, -80);
        while (end_time > TimeNow()
               || encoder_count_limit > robot.movementEncoderCountLeft()
               || encoder_count_limit > robot.movementEncoderCountRight());
        Sleep(100);
    }

    // Move backwards for ___ inches or _ seconds

    // Drop SAM in the hole.

    // Close SAM enclosure.
}
