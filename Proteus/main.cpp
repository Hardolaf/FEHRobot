#include "robot.h"
#include "robotdebug.h"
#include "robotnormal.h"
#include "robotfactory.h"
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
//#include <FEHMOM.h>

void debug_menu();
void performance_test_5(RobotNormal robot);
void line_follow(RobotNormal robot, int timeout);
void competition(RobotNormal robot);
void competition_satellite(RobotNormal robot);

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

        competition(robot);
    } else if (runTimeMode == RobotFactory::RUN_MODE_DEBUG) {
        RobotDebug robot;

        debug_menu();

        while ( true ) {
            if (buttons.LeftPressed()) {
                robot.testMovementForward();
                debug_menu();
            } else if (buttons.MiddlePressed()) {
                robot.testServoElevatorSetAngle();
                debug_menu();
            } else if (buttons.RightPressed()) {
                robot.testMotorSAM();
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
    LCD.WriteLine("Left: Test movement forward");
    LCD.WriteLine("Middle: Test Elevator Servo Set Angle");
    LCD.WriteLine("Right: Tes SAM Motor");
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

    // Ensure that the servos are in the correct start positions
    robot.servoArmHighest();
    Sleep(100);
    robot.servoElevatorLowest();
    Sleep(100);

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
    robot.movementRight(90);
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

/**
 * @brief follow_line This function follows a line
 * @param robot A RobotNormal with RobotNormal::setup() and
 * RobotNormal::calibrate()run
 * @param timeout How long do you want to line follow in ms?
 * @param direction -1 for backwards, 1 for forwards
 */
void follow_line(RobotNormal robot, int timeout, int8 direction) {
    timeout += TimeNowMSec();

    // Follow the line for timeout
    while(TimeNowMSec() < timeout) {
        if (robot.optosensorMiddleSeesLine() == 1) {
            robot.movementMotorManualSet(direction * 80, direction * 40);
        } else {
            robot.movementMotorManualSet(direction * 40, direction * 80);
        }
    }

    // Stop the motors
    robot.movementMotorManualSet(0, 0);
    Sleep(100);
}

/**
 * @brief competition_satellite
 * @param robot A RobotNormal with RobotNormal::setup() and
 * RobotNormal::calibrate()run
 */
void competition_satellite(RobotNormal robot) {
    // Press a button for now.
    // TODO: Implement MOM
    robot.servoElevatorLowest();
    Sleep(100);
    robot.servoArmSetTask();
    Sleep(100);
}

/**
 * @brief competition The competition code. Assumes the robot is facing the
 * right wall.
 * @param robot A RobotNormal with RobotNormal::setup() and
 * RobotNormal::calibrate()run
 * @see startup_sequence()
 */
void competition(RobotNormal robot) {
    // Run the standard startup sequence
    startup_sequence(robot);

    // Enable MOM
    //MOM.Enable();

    // Backup to the wall
    robot.movementMotorManualSet(-80, -80);
    while(!robot.bumpSwitchBackLeftPressed());
    Sleep(400);
    robot.movementMotorManualSet(0, 0);

    // Move to the line.
    robot.movementStraight(63, 8.0);
    Sleep(100);
    robot.movementMotorManualSet(63, 63);
    while(robot.optosensorMiddleSeesLine() != 1);
    robot.movementStraight(63, 7.0);
    robot.movementMotorManualSet(0,0);
    Sleep(100);

    // Follow the line backwards for a bit to line ourselves up and give
    // ourselves room.
    robot.movementLeft(90);
    Sleep(100);

    // Prep for hitting satellite button
    competition_satellite(robot);

    // Follow the line to the button
    follow_line(robot, 650, 1);
    robot.movementMotorManualSet(80,80);
//    while(!robot.bumpSwitchBackRightPressed());
    Sleep(200);
    robot.movementMotorManualSet(0,0);
    Sleep(100);

    // Back up 3 inches
    robot.movementStraight(-80, 3.0);
    Sleep(100);
    robot.servoArmHighest();
    Sleep(300);

    // Turn left
    robot.movementLeft(60);

    // Line up with wall
    robot.movementFrontSquareToWall();

    // Move back a bit
    robot.movementStraight(-63, 0.30);
    Sleep(100);
    robot.movementRight(80);

    // Go up the stairs
    LCD.WriteLine("Forward 30 inches");
    robot.movementStraight(127, 36.0);
    Sleep(100);

    // Turn 45 degress towards wall
    LCD.WriteLine("Left 45 degrees");
    robot.movementLeft(50);
    Sleep(100);

    // Square up against the wall (turn into a function)
    robot.movementFrontSquareToWall();
    Sleep(100);

    // Turn 90 towards pryramid (front facing)
    LCD.WriteLine("Back up 0.25 inches");
    robot.movementStraight(-63, 0.40);
    Sleep(100);
    LCD.WriteLine("Turning right");
    robot.movementRight(90);
    Sleep(100);
    robot.movementFrontSquareToWall();
    Sleep(100);
    robot.movementStraight(-70, 5.0);

    // Set up the arm + elevator
    robot.servoElevatorSetAngle(112);
    Sleep(500);
    robot.servoArmSetTask();

    // Charge the stone like a madman
    bool hitStone = false;
    int charges = 0;
    bool hfr = false;
    while(!hitStone || charges < 2) {
        LCD.WriteLine("Charge STONE");
        double end_time = TimeNow() + 2.0;

        // Move straight forward for 2.0 seconds
        robot.movementMotorManualSet(80, 80);
        while (end_time > TimeNow() && !robot.bumpSwitchBackRightPressed()) {
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

    // Put arm + Elevator down
    robot.servoArmHighest();
    Sleep(300);
    robot.servoElevatorLowest();
    Sleep(300);

    // Drive forward into wall
    robot.movementFrontSquareToWall();
    Sleep(90);

    // Move backwards for ___ inches or _ seconds
    LCD.WriteLine("Performing SAM drop");
    robot.movementMotorManualSet(-80, -80);
    while(!robot.bumpSwitchBackLeftPressed());
    Sleep(100);
    robot.movementMotorManualSet(0, 0);
    Sleep(100);

    // Drop SAM in the hole.
    //robot.motorSAMOpen();
    Sleep(1000);

    // Close SAM enclosure.
    //robot.motorSAMClose();
    Sleep(100);

    // Straight to wall
    robot.movementFrontSquareToWall();

    // Turn 90 degrees right
    robot.movementRight(90);

    // Straight to clear wall (15.0) inches
    robot.movementStraight(80, 15.0);

    // Turn 90 degress left
    robot.movementLeft(90);

    // Forward to wall
    robot.movementFrontSquareToWall();
    Sleep(100);

    // Backup 0.25 in
    //robot.movementStraight(63, 0.25);

    // Right 90
    robot.movementRight(117);

    // Back into wall
    robot.movementMotorManualSet(-100, -80);
    while(!robot.bumpSwitchBackLeftPressed());
    robot.movementMotorManualSet(0, 0);

    // Elevator up
    robot.servoElevatorHighest();
    Sleep(200);

    // Arm down
    robot.servoArmSetTask();
    Sleep(1000);

    // Line follow to SLED
    robot.movementMotorManualSet(63, 63);
    while(robot.optosensorMiddleSeesLine() != 1);
    follow_line(robot, 1500, 1);

    // Elevator down
    robot.servoElevatorLowest();
    Sleep(1000);

    // Drag sled to back wall
    robot.movementMotorManualSet(-80, -80);
    while(!robot.bumpSwitchBackLeftPressed());
    robot.movementMotorManualSet(0, 0);

    // Right 45
    robot.movementRight(55);

    // Move straight ___ in.
    robot.movementStraight(80, 35.0);

    // Arm up
    robot.servoArmHighest();
    Sleep(500);

    // Right 90
    robot.movementRight(80);

    // forward to wall
    robot.movementFrontSquareToWall();

    // left 90
    robot.movementStraight(-63, 0.25);
    robot.movementLeft(90);

    // to the wall
    robot.movementFrontSquareToWall();

    // Right 90
    robot.movementLeft(90);

    // Forward to success!
    robot.movementStraight(90, 14.0);
}
