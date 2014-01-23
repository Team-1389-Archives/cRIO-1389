#include "WPILib.h"
#include "imageanalysisclient.h"
#include <Math.h>
//#include "SpeedController.h"

// XBOX Controller button and axis numbers
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
	
    MyRobotDrive* cDrive; // Custom robotdrive object for 4 jags
    
    //RobotDrive myRobot; // robot drive system
    
    Joystick driveStick, funcStick; // The two XBOX controllers
    
    
    Jaguar jag1;
    Jaguar jag2;
    Jaguar jag3;
    Jaguar jag4;
    ImageAnalysisClient iaClient;
public:
    RobotDemo(): // TODO replace port numbers with define macros
        driveStick(1),
        funcStick(2),
        jag1(1),
        jag2(2),
        jag3(3),
        jag4(4),
        iaClient(IMAGE_ANALYSIS_SERVER_IP, IMAGE_ANALYSIS_SERVER_PORT){

        //jag2(2,CANJaguar::kVoltage);
        //jag3(3,CANJaguar::kVoltage);
        //jag4(4,CANJaguar::kVoltage);
        //myRobot(&jag1,&jag2,&jag3,&jag4);
        //stick(1);        // as they are declared above.
        cDrive = new MyRobotDrive(&jag1,&jag2,&jag3,&jag4);
        cDrive->SetExpiration(0.1);
        //myRobot.SetExpiration(0.1);
        //myRobot.SetInvertedMotor(RobotDrive::kFrontLeftMotor,true);
        //myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor,true);
    }

    void Autonomous() {
//These macros are for testing purposes only 
    				// Why?
    	
#define CENTER_LINE         (320)
#define CENTER_THRESHOLD    (20)
#define RADIUS_TARGET       (60)
#define RADIUS_THRESHOLD    (20)
        while(IsAutonomous()) {
            ImageData data;
            iaClient.copyImageData(&data);
            float move=0;
            float rotate=0;
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
            cDrive->ArcadeDrive(move, rotate);
        }
    }


    /**
     * Runs the motors with arcade steering.
     */
    void OperatorControl() {
        //    myRobot.SetSafetyEnabled(true);
    	
    	double x, y;
    	
        while (IsOperatorControl()) {
        	
        	x=driveStick.GetRawAxis(LeftX);
        	y=-driveStick.GetRawAxis(LeftY); // The xbox controller uses down as positive for joysticks
        	
        	// Using ArcadeDrive with two numbers (move and rotate) works better than passing
        	//		the xbox joystick object, and is easier to modify to apply a speed modifier.
            cDrive->ArcadeDrive(x, y);
            
            
            Wait(0.005);                // wait for a motor update time
        }
        
        //delete(cDrive); // TODO verify commenting this line is correct
        // Unsure that we need to delete cDrive, and suspect it is causing issues
        // 		where robot does not drive after teleop is disabled and re-enabled, requiring a reboot
    }

    /**
     * Runs during test mode
     */
    void Test() {
    	
    }
};

START_ROBOT_CLASS(RobotDemo);
