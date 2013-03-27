#include "robot.h"
#include "robotdebug.h"
#include "robotnormal.h"
#include "robotfactory.h"
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>

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
        LCD.Clear( FEHLCD::Black );
        LCD.WriteLine("DEBUG MODE");
        LCD.WriteLine("Left: Calibrate Encoders");
        Sleep( 300 );
        while ( true ) {
            if (buttons.LeftPressed()) {
                robot.calibrateEncoders();
            }

            Sleep( 100 );
        }
    }

    return 0;
}

