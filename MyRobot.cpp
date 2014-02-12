#include "WPILib.h"
#include "imageanalysisclient.h"
#include <Math.h>
//#include "SpeedController.h"

#define Pi					(3.1415926535898) // Pi to 14 places
#define Tao					(2 * Pi) // At preference of user
#define Octopi				(8 * Pi) // "                   "

// XBOX Controller button and axis numbers
#define ControllerA			(1) // Port for primary controller, drive controller
#define ControllerB			(2) // Port for secondary controller, functionality

#define ButtonX 			(3) // XBox Controller X Button number for getRawButton() or getRawAxis()
#define ButtonA				(1) // XBox Controller A Button number
#define ButtonB				(2) // XBox Controller B Button number
#define ButtonY				(4) // XBox Controller Y Button number
#define BumperL				(5) // XBox Controller  Left Bumper number
#define BumperR				(6) // XBox Controller Right Bumper number

#define LeftY				(2) // XBox Controller  Left Y Axis number
#define LeftX				(1) // XBox Controller  Left X Axis number
#define RightY				(5) // XBox Controller Right Y Axis number
#define RightX				(4) // XBox Controller Right X Axis number

// PID Starting values
// TODO test for new ones, these ones borrowed from 2012 & 2013
#define PID_P				(1.505)
#define PID_I				(0.003)
#define PID_D				(0.000)


class RobotDemo : public SimpleRobot {
	
    
    Joystick driveStick, funcStick; // The two XBOX controllers
    

    Victor *kickerL, *kickerR;
    
    
    // These save time typing out DriverStationLCD repeatedly for displays
    DriverStationLCD *display;
    DriverStationLCD::Line line1, line2, line3, line4, line5, line6;
    
public:
    RobotDemo():
        driveStick(ControllerA),
        funcStick(ControllerB){

    	kickerL=new Victor(1);//
    	kickerR=new Victor(2); 
    	
        
        // Driverstation Display shortcuts
        display=DriverStationLCD::GetInstance();
        line1=DriverStationLCD::kUser_Line1;
        line2=DriverStationLCD::kUser_Line2;
        line3=DriverStationLCD::kUser_Line3;
        line4=DriverStationLCD::kUser_Line4;
        line5=DriverStationLCD::kUser_Line5;
        line6=DriverStationLCD::kUser_Line6;
        
        // Print version info
        display->PrintfLine(line2, "Kick it");
        display->UpdateLCD();        
        //Preferences::GetInstance()->PutInt("TestNumber", 1);
        display->PrintfLine(line3, "uwu");
        display->UpdateLCD();
                
    }

    void Autonomous() {
//These macros are for testing purposes only 
    				// Why?
    	
        while(IsAutonomous()&&IsEnabled()) {
            display->PrintfLine(line2, "Kicker doesn't move autonomously");

            display->UpdateLCD();
            
        }
        
        
    }


    void OperatorControl() {
    	
        while (IsOperatorControl()&&IsEnabled()) {
        	display->PrintfLine(line1, "Telobop");
        	
        	KickerTest();
    		
            display->UpdateLCD();
            display->Clear();
            
        }

        
    }
    
    
    void KickerTest(){
    	float value=driveStick.GetRawAxis(RightY);
    	if(fabs(value)<0.08)
    		value=0;
    	kickerL->Set(value);
    	kickerR->Set(value);
    	
    	
    	display->PrintfLine(line2, "Kick: %f", value);
    	
    }
    
    /**
     * Runs during test mode
     */
    void Test() {
    	while(IsTest()&&IsEnabled()){
    		
    		display->PrintfLine(line1, "No test set.");
    		
    		display->UpdateLCD();
    	}


    }
};

START_ROBOT_CLASS(RobotDemo);
