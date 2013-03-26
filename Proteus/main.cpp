#include "robot.h"
#include "robotdebug.h"
#include "robotnormal.h"
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

    
    while( true )
    {
        if( buttons.LeftPressed() ) {
            LCD.WriteLine( "Not implemented." );
            Sleep( 100 );
        } else if ( buttons.RightPressed()) {
            LCD.Clear( FEHLCD::Black );
            Sleep( 300 );
            while ( true ) {
                LCD.Clear( FEHLCD::Black );
                LCD.WriteLine("DEBUG MODE");
                LCD.WriteLine("Left: Calibrate Encoders");
                RobotDebug robot = new RobotDebug();

                if (buttons.LeftPressed()) {
                    robot.calibrateEncoders();
                    Sleep ( 100 );
                }

                Sleep( 100 );
            }
        }
    }
    return 0;
}

