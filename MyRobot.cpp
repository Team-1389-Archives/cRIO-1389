#include "WPILib.h"
#include "imageanalysisclient.h"
#include <Math.h>
//#include "SpeedController.h"

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


class MyRobotDrive : public RobotDrive {
    //CANJaguar* jag1, jag2, jag3, jag4;
    Jaguar* jag1;
    Jaguar* jag2;
    Jaguar* jag3;
    Jaguar* jag4;

public: 
    
	// RobotDrive can already accept four jaguars as its motors, this custom class seems unnecessary

    /* (front) left, (rear) left, (front) right, (rear) right */
    
    MyRobotDrive(Jaguar* j1,Jaguar* j2,Jaguar* j3,Jaguar* j4):
    	
        RobotDrive((SpeedController *)NULL,(SpeedController *)NULL) {
        jag1 = j1;
        jag2 = j2;
        jag3 = j3;
        jag4 = j4;
    }

    void SetLeftRightMotorOutputs(float leftOutput, float rightOutput) {
        jag1->Set(leftOutput);
        jag2->Set(leftOutput);
        jag3->Set(rightOutput);
        jag4->Set(rightOutput);
    }
};

class RobotDemo : public SimpleRobot {
	
    RobotDrive* cDrive; // Generic robotdrive object for 4 jags
    
    
    Joystick driveStick, funcStick; // The two XBOX controllers
    
    
    Jaguar jag1;
    Jaguar jag2;
    Jaguar jag3;
    Jaguar jag4;
    ImageAnalysisClient iaClient;
    
    
    // These save time typing out DriverStationLCD repeatedly for displays
    DriverStationLCD *display;
    DriverStationLCD::Line line1, line2, line3, line4, line5, line6;
    
public:
    RobotDemo(): // TODO replace port numbers with define macros
        driveStick(ControllerA),
        funcStick(ControllerB),
        jag1(1),
        jag2(2),
        jag3(3),
        jag4(4),
        iaClient(IMAGE_ANALYSIS_SERVER_IP, IMAGE_ANALYSIS_SERVER_PORT){

        //jag2(2,CANJaguar::kVoltage);
        //jag3(3,CANJaguar::kVoltage);
        //jag4(4,CANJaguar::kVoltage);
        cDrive = new MyRobotDrive(&jag1,&jag2,&jag3,&jag4);
        cDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor,true);
        cDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor,true);
        cDrive->SetExpiration(0.1);
        
        display=DriverStationLCD::GetInstance();
        line1=DriverStationLCD::kUser_Line1;
        line2=DriverStationLCD::kUser_Line2;
        line3=DriverStationLCD::kUser_Line3;
        line4=DriverStationLCD::kUser_Line4;
        line5=DriverStationLCD::kUser_Line5;
        line6=DriverStationLCD::kUser_Line6;
        
        display->PrintfLine(line1, "Test...");
        display->UpdateLCD();
        //SmartDashboard::PutNumber("TestNumber", 1);
                
    }

    void Autonomous() {
//These macros are for testing purposes only 
    				// Why?
    	
#define CENTER_LINE         (320)
#define CENTER_THRESHOLD    (20)
#define RADIUS_TARGET       (60)
#define RADIUS_THRESHOLD    (20)
        while(IsAutonomous()&&IsEnabled()) {
            ImageData data;
            iaClient.copyImageData(&data);
            display->PrintfLine(line2, "IA: %d, %d, %d", (int)data.x, (int)data.y, (int)data.radius);

            float move=0;
            float rotate=0;
            
            /*
            if(fabs(data.x-CENTER_LINE)>CENTER_THRESHOLD) {
                if(data.x<CENTER_LINE) {
                    rotate=1.0;
                } else {
                    rotate=-1.0;
                }
            }
            if(fabs(data.radius-RADIUS_TARGET)>RADIUS_THRESHOLD) {
                if(data.radius>RADIUS_THRESHOLD) {
                    move=1.0;
                } else {
                    move=-1.0;
                }
            }
            
            display->PrintfLine(line3, "Move: %d", move);
            display->PrintfLine(line4, "Rotate: %d", rotate);
            //*/
            double test=1;//SmartDashboard::GetNumber("TestNumber");
            
            display->PrintfLine(line3, "Test: %f", test);
                        
            if(test==1)
            	move=1;
            if(test==2)
            	rotate=1;
            display->PrintfLine(line4, "Move: %f", move);
            display->PrintfLine(line5, "Rotate: %f", rotate);
                        
            display->UpdateLCD();
            cDrive->ArcadeDrive(move, rotate);
            
        }
        
        jag1.Set(0);
        jag2.Set(0);
        jag3.Set(0);
        jag4.Set(0);
        
    }


    /**
     * Runs the motors with arcade steering.
     */
    void OperatorControl() {
        //    myRobot.SetSafetyEnabled(true);
    	
    	// TODO consider floats vs doubles
    	double x, y;
    	double speedMod; // Speed modifier
    	
        while (IsOperatorControl()&&IsEnabled()) {
        	speedMod=.65;
        	if(driveStick.GetRawButton(BumperR)) // Hold Right bumper to go at full speed
        		speedMod=1;
        	if(driveStick.GetRawButton(BumperL)) // Hold Left bumper to go at 30% or 1/2 normal speed
        		speedMod=.5; // Is checked second so in case both bumpers are held, slower speed is used
        	
        	x=driveStick.GetRawAxis(LeftX);
        	y=-driveStick.GetRawAxis(LeftY); // The xbox controller uses down as positive for joysticks
        	
        	
        	// Using ArcadeDrive with two numbers (move and rotate) works better than passing
        	//		the xbox joystick object, and is easier to modify to apply a speed modifier.
            
        	// TODO use move then rotate, not rotate then move
        	// TODO fix one drive side being inverted
        	cDrive->ArcadeDrive(speedMod*y, speedMod*x);

        	double test=1;//SmartDashboard::GetNumber("TestNumber");
        	
        	display->PrintfLine(line1, "Teleop");
        	display->PrintfLine(line2, "TestNumber: %f", test);
            display->PrintfLine(line3, "Move: %f", speedMod*y);
            display->PrintfLine(line4, "Rotate: %f", speedMod*x);
                        
            display->UpdateLCD();
            
            Wait(0.005);                // wait for a motor update time
        }

        jag1.Set(0);
        jag2.Set(0);
        jag3.Set(0);
        jag4.Set(0);
        
    }

    /**
     * Runs during test mode
     */
    void Test() {
    	while(IsTest()&&IsEnabled()){

    		cDrive->ArcadeDrive(.6, 0); // Test if the robot drives in the right direction forward
            
    	}

    	
        jag1.Set(0);
        jag2.Set(0);
        jag3.Set(0);
        jag4.Set(0);
        
    }
};

START_ROBOT_CLASS(RobotDemo);
