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

// CAN Jaguar Numbers
#define CanNumLF			(1) //  Left Front motor CAN number
#define CanNumLR			(2) //  Left  Rear motor CAN number
#define CanNumRF			(3) // Right Front motor CAN number
#define CanNumRR			(4) // Right  Rear motor CAN number
#define CanNumKick			(6) // Kicker motor CAN number
#define CanNumRamp			(7) // Ramp motor CAN number

// Drive Motor Ports
#define DriveMotorLF		(1) //  Left Front motor port
#define DriveMotorLR		(2) //  Left  Rear motor port
#define DriveMotorRF		(3) // Right Front motor port
#define DriveMotorRR		(4) // Right  Rear motor port

// Victor (pwm) ports
#define VicTower			(1) // Tower motor PWM port

// TODO remove Encoder macros when we verify we can just use CANJaguar motor control
// Encoder Channel numbers
#define EncoderLA			(1) //  Left encoder channel A number
#define EncoderLB			(2) // Right encoder channel B number
#define EncoderRA			(3) //  Left encoder channel A number
#define EncoderRB			(4) // Right encoder channel B number


// Robot physical data
#define WheelCircumference	(Pi / 2) // Circumference in feet (6 in diameter)


// PID Starting values
// TODO test for new ones, these ones borrowed from 2012 & 2013
#define PID_P				(1.505)
#define PID_I				(0.003)
#define PID_D				(0.000)

// Encoder data
#define EncoderPulses		(360) // Number of pulses per rotation on the drive encoders
#define KickerPulses		(1800) // Number of pulses per rotation on the kicker encoder

class EncodedRobotDrive : public RobotDrive {
    //CANJaguar* jag1, jag2, jag3, jag4;
    CANJaguar* jag1;
    CANJaguar* jag2;
    CANJaguar* jag3;
    CANJaguar* jag4;
    
    float speed;

public: 
    
	// RobotDrive can already accept four jaguars as its motors, this custom class seems unnecessary

    /* (front) left, (rear) left, (front) right, (rear) right */
    
    EncodedRobotDrive(CANJaguar* j1,CANJaguar* j2,CANJaguar* j3,CANJaguar* j4):
    	
        RobotDrive((SpeedController *)NULL,(SpeedController *)NULL) {
        jag1 = j1;
        jag2 = j2;
        jag3 = j3;
        jag4 = j4;
        speed=10;
    }
    
    void SetMaxSpeed(float maxSpeed){
    	speed=maxSpeed;
    }

    void SetLeftRightMotorOutputs(float leftOutput, float rightOutput) {
    	leftOutput*=1;//speed; 
    	rightOutput*=1;//speed;
        jag1->Set(leftOutput);
        jag2->Set(leftOutput);
        jag3->Set(rightOutput);
        jag4->Set(rightOutput);
        
        DriverStationLCD::GetInstance()->PrintfLine(DriverStationLCD::kUser_Line5, "Target: %f", leftOutput);
    }
};


class RobotDemo : public SimpleRobot {
	
    EncodedRobotDrive* cDrive; // Generic robotdrive object for 4 jags
    
    
    Joystick driveStick, funcStick; // The two XBOX controllers
    
    ImageAnalysisClient iaClient;

    CANJaguar *driveLF, *driveLR, *driveRF, *driveRR;
    //CANJaguar *kicker, *ramp;
    
    Victor *tower;
    
    // These save time typing out DriverStationLCD repeatedly for displays
    DriverStationLCD *display;
    DriverStationLCD::Line line1, line2, line3, line4, line5, line6;
    
public:
    RobotDemo():
        driveStick(ControllerA),
        funcStick(ControllerB),
        iaClient(IMAGE_ANALYSIS_SERVER_IP, IMAGE_ANALYSIS_SERVER_PORT){

    	// CANJaguar setup 
    	driveLF=new CANJaguar(CanNumLF);//, CANJaguar::kSpeed);
    	driveLR=new CANJaguar(CanNumLR);//, CANJaguar::kSpeed);
    	driveRF=new CANJaguar(CanNumRF); // Temporarily no encoders on Right side
    	driveRR=new CANJaguar(CanNumRR);
    	//kicker=new CANJaguar(CanNumKick);//, CANJaguar::kPosition);
    	//ramp=new CANJaguar(CanNumRamp);
    	
    	/*
    	driveLF->SetPID(PID_P, PID_I, PID_D);
    	driveLR->SetPID(PID_P, PID_I, PID_D);
    	driveRF->SetPID(PID_P, PID_I, PID_D);
    	driveRR->SetPID(PID_P, PID_I, PID_D);

    	driveLF->ConfigEncoderCodesPerRev(EncoderPulses); // Assume pulses = Codes
    	driveLR->ConfigEncoderCodesPerRev(EncoderPulses);
    	driveRF->ConfigEncoderCodesPerRev(EncoderPulses);
    	driveRR->ConfigEncoderCodesPerRev(EncoderPulses);
    	
    	driveLF->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
    	driveLR->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
    	driveRF->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
    	driveRR->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
    	
    	driveLF->EnableControl();
    	driveLR->EnableControl();
    	driveRF->EnableControl();
    	driveRR->EnableControl(); *///
    	
    	// Init Robotdrive
    	cDrive = new EncodedRobotDrive(driveLF, driveLR, driveRF, driveRR);
    	cDrive->SetMaxSpeed(10); // Set max speed to 10 ft/s default
        cDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor,false);
        cDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor,false);
        cDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor,false);
        cDrive->SetInvertedMotor(RobotDrive::kRearRightMotor,false);
        cDrive->SetExpiration(0.1);
        
        // Victor setup
        tower=new Victor(VicTower);
        
        
        // Driverstation Display shortcuts
        display=DriverStationLCD::GetInstance();
        line1=DriverStationLCD::kUser_Line1;
        line2=DriverStationLCD::kUser_Line2;
        line3=DriverStationLCD::kUser_Line3;
        line4=DriverStationLCD::kUser_Line4;
        line5=DriverStationLCD::kUser_Line5;
        line6=DriverStationLCD::kUser_Line6;
        
        // Print version info
        display->PrintfLine(line2, "Client test");
        display->UpdateLCD();        
        //Preferences::GetInstance()->PutInt("TestNumber", 1);
        display->PrintfLine(line3, "^w^");
        display->UpdateLCD();
                
    }

    void Autonomous() {
//These macros are temporary
#define CENTER_LINE         (320)
#define CENTER_THRESHOLD    (20)
#define RADIUS_TARGET       (60)
#define RADIUS_THRESHOLD    (20)
    	
    	// TODO set cDrive max speed to desired value.
        while(IsAutonomous()&&IsEnabled()) {
            ImageData data;
            iaClient.copyImageData(&data);
            display->PrintfLine(line2, "X: %f", data.x);
            display->PrintfLine(line3, "Y: %f", data.y);
            display->PrintfLine(line4, "R: %f", data.radius);
            display->PrintfLine(line5, "(^w^)");
            display->UpdateLCD();
            
        } 
        
        driveLF->Set(0);
        driveLR->Set(0);
        driveRF->Set(0);
        driveRR->Set(0);
        
    }


    void OperatorControl() {
        //    myRobot.SetSafetyEnabled(true);
    			// Is this important?
    	
    	
    	// TODO set cDrive max speed to a desirable value
    	
        while (IsOperatorControl()&&IsEnabled()) {
        	display->PrintfLine(line1, "Teleop");
        	
        	DriveIterate();
        	TowerTest();
    		
            display->UpdateLCD();
            display->Clear();
            
        }

        driveLF->Set(0);
        driveLR->Set(0);
        driveRF->Set(0);
        driveRR->Set(0);
        
    }
    
    void TowerTest(){
    	float value=driveStick.GetRawAxis(RightY);
    	if(fabs(value)<0.08)
    		value=0;
    	
    	tower->Set(value);

        display->PrintfLine(line6, "Blocker: %f", value);
    }
    
    void DriveIterate(){
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
    	cDrive->ArcadeDrive(speedMod*y, speedMod*x);
    	
        display->PrintfLine(line2, "Move: %f", speedMod*y);
        if(lock)
        	display->PrintfLine(line3, "Rotation locked");
        else display->PrintfLine(line3, "Rotate: %f", speedMod*x);
        display->PrintfLine(line4, "Encode: %f", (float)driveLF->GetSpeed());
    }

    
    /**
     * Runs during test mode
     */
    void Test() {
    	while(IsTest()&&IsEnabled()){
    		
    		display->PrintfLine(line2, "No test set.");
    		
    		display->UpdateLCD();
    	}


        driveLF->Set(0);
        driveLR->Set(0);
        driveRF->Set(0);
        driveRR->Set(0);
        
    }
};

START_ROBOT_CLASS(RobotDemo);
