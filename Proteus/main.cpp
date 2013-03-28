#include "robot.h"
#include "robotdebug.h"
#include "robotnormal.h"
#include "robotfactory.h"
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>

void debug_menu();

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
            LCD.WriteLine( "Not implemented." );
            Sleep( 100 );
        } else if ( buttons.RightPressed()) {
            runTimeMode = RobotFactory::RUN_MODE_DEBUG;
            break;
        }
    }

    if (runTimeMode == RobotFactory::RUN_MODE_NORMAL) {
        RobotNormal robot;
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
