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
	
<<<<<<< HEAD
    //RobotDrive* cDrive; // Generic robotdrive object for 4 jags
    PIDRobotDrive* pDrive; // PID controlled drive object for 4 PIDControllers
=======
    RobotDrive* cDrive; // Generic robotdrive object for 4 jags
>>>>>>> parent of 18423cb... Added PIDRobotDrive class and Encoders
    
    
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

<<<<<<< HEAD
    	/*   // CANJaguar setup
    	driveLF=new CANJaguar(CanNumLF, CANJaguar::kSpeed);
    	driveLR=new CANJaguar(CanNumLR, CANJaguar::kSpeed);
    	driveRF=new CANJaguar(CanNumRF, CANJaguar::kSpeed);
    	driveRR=new CANJaguar(CanNumRR, CANJaguar::kSpeed);

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
    	driveRR->EnableControl(); 
    	//*/

    	driveLF=new Victor(DriveMotorLF);
    	driveLR=new Victor(DriveMotorLR);
    	driveRF=new Victor(DriveMotorRF);
    	driveRR=new Victor(DriveMotorRR);
    	
    	// Encoder port macros are -1 until we learn actual ports
    	encoderL=new Encoder(EncoderLA, EncoderLB, false); // The false is on reversing directions
    	encoderR=new Encoder(EncoderRA, EncoderRB, false);
    	
    	// Set encoders to be controlled by rate, not distance
    	// Perhaps distance is preferable?
    	encoderL->SetPIDSourceParameter(PIDSource::kRate);
    	encoderR->SetPIDSourceParameter(PIDSource::kRate);
    	
    	
    	// Sets the distance of one pulse so that a full rotation of pulses is the distance driven
    	encoderL->SetDistancePerPulse( WheelCircumference / EncoderPulses );
    	encoderR->SetDistancePerPulse( WheelCircumference / EncoderPulses );

    	pidLF=new PIDController(PID_P, PID_I, PID_D, encoderL, driveLF);
    	pidLR=new PIDController(PID_P, PID_I, PID_D, encoderL, driveLR);
    	pidRF=new PIDController(PID_P, PID_I, PID_D, encoderR, driveRF);
    	pidRR=new PIDController(PID_P, PID_I, PID_D, encoderR, driveRR);

    	pidLF->Enable();
    	pidLR->Enable();
    	pidRF->Enable();
    	pidRR->Enable();
    	
    	pDrive = new PIDRobotDrive(pidLF, pidLR, pidRF, pidRR);
        pDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor,false);
        pDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor,false);
        pDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor,false);
        pDrive->SetInvertedMotor(RobotDrive::kRearRightMotor,false);
        pDrive->SetExpiration(0.1);
        pDrive->SetMaxSpeed(5); // Temp max speed of 10 feet per second
    	
    	
        //jag2(2,CANJaguar::kVoltage);
        //jag3(3,CANJaguar::kVoltage);
        //jag4(4,CANJaguar::kVoltage);
    	
    	/*
        cDrive = new RobotDrive(driveLF,driveLR,driveRF,driveRR);
=======
        //jag2(2,CANJaguar::kVoltage);
        //jag3(3,CANJaguar::kVoltage);
        //jag4(4,CANJaguar::kVoltage);
        cDrive = new RobotDrive(&jag1,&jag2,&jag3,&jag4);
>>>>>>> parent of 18423cb... Added PIDRobotDrive class and Encoders
        cDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor,false);
        cDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor,false);
        cDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor,false);
        cDrive->SetInvertedMotor(RobotDrive::kRearRightMotor,false);
        cDrive->SetExpiration(0.1);//*/
        
        display=DriverStationLCD::GetInstance();
        line1=DriverStationLCD::kUser_Line1;
        line2=DriverStationLCD::kUser_Line2;
        line3=DriverStationLCD::kUser_Line3;
        line4=DriverStationLCD::kUser_Line4;
        line5=DriverStationLCD::kUser_Line5;
        line6=DriverStationLCD::kUser_Line6;
        
        
        display->PrintfLine(line2, "More prefs");
        display->UpdateLCD();        
        Preferences::GetInstance()->PutInt("TestNumber", 1);
        Preferences::GetInstance()->PutBoolean("TestBool", false);
        display->PrintfLine(line3, "Put data");
        display->UpdateLCD();
                
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
            int test=Preferences::GetInstance()->GetInt("TestNumber");
            
            display->PrintfLine(line3, "Test: %d", test);
                        
            if(test==1)
            	move=1;
            if(test==2)
            	rotate=1;
            display->PrintfLine(line4, "Move: %f", move);
            display->PrintfLine(line5, "Rotate: %f", rotate);
                        
            display->UpdateLCD();
            pDrive->ArcadeDrive(move, rotate);
            
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
        	
        	x=-driveStick.GetRawAxis(LeftX); 
        	  // Inverting x because the robot was turning the wrong way
        	
        	y=-driveStick.GetRawAxis(LeftY); 
        	  // The xbox controller uses down as positive for joysticks
        	
        	
        	// Using ArcadeDrive with two numbers (move and rotate) works better than passing
        	//		the xbox joystick object, and is easier to modify to apply a speed modifier.
            
        	pDrive->ArcadeDrive(speedMod*y, speedMod*x);

        	int test=Preferences::GetInstance()->GetInt("TestNumber");
        	
        	display->PrintfLine(line1, "Teleop");
        	display->PrintfLine(line2, "TestNumber: %d", test);
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
    		
    		Preferences *prefs=Preferences::GetInstance();
    		int num=prefs->GetInt("TestNumber");
    		bool boo=prefs->GetBoolean("TestBool");
    		display->PrintfLine(line2, "Num = %d", num);
    		display->PrintfLine(line3, "No boo :c");
    		if(boo)
    			display->PrintfLine(line3, "Boo! >:3 ");
    		
    		display->UpdateLCD();
    	}

    	
        jag1.Set(0);
        jag2.Set(0);
        jag3.Set(0);
        jag4.Set(0);
        
    }
};

START_ROBOT_CLASS(RobotDemo);
