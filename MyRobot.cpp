#include "WPILib.h"
#include "imageanalysisclient.h"
#include <Math.h>
//#include "SpeedController.h"
#include "controller-macros.h"
#include "robot-data.h"

#define PID_P				(1.505)
#define PID_I				(0.003)
#define PID_D				(0.000)

#define EncoderPulses		(360) // Number of pulses per rotation on the drive encoders
#define KickerPulses		(1800) // Number of pulses per rotation on the kicker encoder

#define DRIVE_TRAIN_MOTORS_CONTROL_TYPE		(CANJaguar::kPercentVbus)
struct DriveTrainMotors{
	DriveTrainMotors(int frontLeftId, int rearLeftId, int frontRightId, int rearRightId)
		: frontLeft(frontLeftId, DRIVE_TRAIN_MOTORS_CONTROL_TYPE),
		  rearLeft(rearLeftId, DRIVE_TRAIN_MOTORS_CONTROL_TYPE),
		  frontRight(frontRightId, DRIVE_TRAIN_MOTORS_CONTROL_TYPE),
		  rearRight(rearRightId, DRIVE_TRAIN_MOTORS_CONTROL_TYPE)
	{}
	virtual ~DriveTrainMotors(){}
	CANJaguar frontLeft, rearLeft, frontRight, rearRight;
};

class RobotDemo : public SimpleRobot {
    Joystick driveStick, funcStick; // The two XBOX controllers
    ImageAnalysisClient iaClient;
    DriverStationLCD *display;
    DriveTrainMotors drive_train_motors;
    RobotDrive drive;
    Encoder testEncoder;
    Victor *blocker;
public:
    RobotDemo():
        driveStick(ControllerA),
        funcStick(ControllerB),
        iaClient(IMAGE_ANALYSIS_SERVER_IP, IMAGE_ANALYSIS_SERVER_PORT),
        drive_train_motors(1,2,3,4),//check these ports
        
        drive(
        		drive_train_motors.frontLeft, drive_train_motors.rearLeft,
        		drive_train_motors.frontRight, drive_train_motors.rearRight
        ),
        testEncoder(EncoderTestA, EncoderTestB, false, Encoder::k4X)
    	// Assuming 4X encoding
    {
    	blocker=new Victor(6);
    	
    	testEncoder.SetPIDSourceParameter(PIDSource::kRate);
    	testEncoder.SetDistancePerPulse(WheelCircumference / EncoderPulses);
    	
    	testEncoder.Start();
    	
    	
        display=DriverStationLCD::GetInstance();
        
        display->PrintfLine(DriverStationLCD::kUser_Line2, "Restarted");
        display->PrintfLine(DriverStationLCD::kUser_Line3, "Blocker Test4");
        display->UpdateLCD();
                
    }

    void Autonomous() {
    	
    	// TODO set cDrive max speed to desired value.
        while(IsAutonomous()&&IsEnabled()) {
            ImageData data;
            iaClient.copyImageData(&data);
            display->PrintfLine(DriverStationLCD::kUser_Line2, "X: %f", data.x);
            display->PrintfLine(DriverStationLCD::kUser_Line3, "Y: %f", data.y);
            display->PrintfLine(DriverStationLCD::kUser_Line4, "R: %f", data.radius);
         
            display->UpdateLCD();
            
            if (data.radius > 10.0f){
            	if (data.x < 150){
            		drive.ArcadeDrive(0.5f, 0.0f);
            		continue;
            	}else if (data.x > 170){
            		drive.ArcadeDrive(-0.5f, 0.0f);
            		continue;
            	}
            }
            drive.ArcadeDrive(0.0f, 0.0f);
            
            
        } 
        
    }


    void OperatorControl() {
    	while (IsOperatorControl()&&IsEnabled()) {
        	display->PrintfLine(DriverStationLCD::kUser_Line1, "Teleop");
        	
        	Drivetrain();
        	EncoderTest();
        	TowerTest();
        	
            display->UpdateLCD();
            Wait(0.005);
        }
        drive.ArcadeDrive(0.0f, 0.0f);
    }
    
    void Drivetrain(){
    	float x, y;
    	float speedMod; // Speed modifier
    	speedMod=.65;
    	if(driveStick.GetRawButton(BumperR)) // Hold Right bumper to go at full speed
    		speedMod=1;
    	if(driveStick.GetRawButton(BumperL)) // Hold Left bumper to go at 30% or 1/2 normal speed
    		speedMod=.5; // Is checked second so in case both bumpers are held, slower speed is used
    	
    	x=-driveStick.GetRawAxis(LeftX); 
    	  // Inverting x because the robot was turning the wrong way
    	
    	y=-driveStick.GetRawAxis(LeftY); 
    	  // The xbox controller uses down as positive for joysticks
    	
    	
    	bool lock=driveStick.GetRawButton(ButtonA);
    	
    	if(lock)
    		x=0;
    	drive.ArcadeDrive(speedMod*y, speedMod*x);
    	
        display->PrintfLine(DriverStationLCD::kUser_Line2, "Move: %f", speedMod*y);
        if(lock)
        	display->PrintfLine(DriverStationLCD::kUser_Line3, "Rotation locked");
        else
        	display->PrintfLine(DriverStationLCD::kUser_Line3, "Rotate: %f", speedMod*x);
    }
    
    void EncoderTest(){
    	if(driveStick.GetRawButton(ButtonB))
    		testEncoder.Reset();
    	display->PrintfLine(DriverStationLCD::kUser_Line4, "Enc: %f", (float)testEncoder.GetDistance());
    }
    void TowerTest(){
    	if(driveStick.GetRawButton(ButtonX)){
    		blocker->Set(1);
        	display->PrintfLine(DriverStationLCD::kUser_Line5, "On");
    	} else{
        	display->PrintfLine(DriverStationLCD::kUser_Line5, "Off");
    		blocker->Set(0);
    	}
    }
    
    /**
     * Runs during test mode
     */
    void Test() {
    	while(IsTest()&&IsEnabled()){
    		display->PrintfLine(DriverStationLCD::kUser_Line2, "No test set.");
    		display->UpdateLCD();
    	}
    }
};

START_ROBOT_CLASS(RobotDemo);
