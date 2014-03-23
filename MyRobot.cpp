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

struct KickerMotors{
	KickerMotors(int leftId, int rightId)
	: left(leftId),
	right(rightId)
	{}
	virtual ~KickerMotors(){}
	CANJaguar left, right;
};



class RobotDemo : public SimpleRobot {
	Joystick driveStick, funcStick; // The two XBOX controllers
	ImageAnalysisClient iaClient;
	DriverStationLCD *display;
	DriveTrainMotors drive_train_motors;
	//KickerMotors kicker_motors;
	RobotDrive drive;
	Encoder testEncoder;
	DigitalModule *digi;
	Victor *blocker;
	bool lowered;
	bool rollerOn;
	bool buttonBDown;
	bool umbrellaUp, xPressed, goDown;
	int buttonPresses;
	Victor rampV;
	Victor rollerV;
	DoubleSolenoid rampPnumatic1;
	DoubleSolenoid rampPnumatic2;
	DoubleSolenoid umbrella;
public:
	RobotDemo():
		driveStick(ControllerA),
		funcStick(ControllerB),
		iaClient(IMAGE_ANALYSIS_SERVER_IP, IMAGE_ANALYSIS_SERVER_PORT),
		drive_train_motors(1,2,3,4),//check these ports
		//kicker_motors(5,6),
		drive(
				drive_train_motors.frontLeft, drive_train_motors.rearLeft,
				drive_train_motors.frontRight, drive_train_motors.rearRight
		),
		testEncoder(EncoderTestA, EncoderTestB, false, Encoder::k4X),
		// Assuming 4X encoding
		rampV(1, RampVictor),
		rollerV(1, RollerVictor),
		rampPnumatic1(RollerPnumaticA1,RollerPnumaticB1),
		rampPnumatic2(RollerPnumaticA2,RollerPnumaticB2),
		umbrella(UmbrellaSolenoidOn, UmbrellaSolenoidOff)
		{
		digi=DigitalModule::GetInstance(1);

		blocker=new Victor(1, 9);
		buttonPresses = 0;
		testEncoder.SetPIDSourceParameter(PIDSource::kRate);
		testEncoder.SetDistancePerPulse(WheelCircumference / EncoderPulses);

		testEncoder.Start();

		lowered=true;

		display=DriverStationLCD::GetInstance();

		display->PrintfLine(DriverStationLCD::kUser_Line2, "Restarted");
		display->PrintfLine(DriverStationLCD::kUser_Line3, "Blocker v5");
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
				if (data.x < 120){
					drive.ArcadeDrive(-0.5f, 0.0f);
					continue;
				}else if (data.x > 200){
					drive.ArcadeDrive(0.5f, 0.0f);
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
			//EncoderTest();
			TowerTest();
			//KickerTest();
			umbrellaMakerAfier();
			ramp();
			roller();

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
		bool top=digi->GetDIO(12);
		bool bottom=digi->GetDIO(10);
		float tower=0;
		if(driveStick.GetRawButton(ButtonX)){ // If told to raise
			lowered=bottom;
			if(top){ // If touching the top
				tower=.2;
			} else{
				tower=1;
			}
		} else{
			if(lowered){ // If has hit the bottom
				tower=0;
			} else{
				tower=-.2; // Move direction opposite to raising the blocker
				lowered=bottom; // Set has been lowered to current bottom limit setting
			}
		}
		display->PrintfLine(display->kUser_Line5, "T: %f", tower);
		blocker->Set(tower);
	}

	void KickerTest(){

		float kick=driveStick.GetRawAxis(RightY);
		if(fabs(kick)<0.08)
			kick=0;
		if(kick>0.9)
			kick = 1;
		if(kick<-0.9)
			kick = -1;
		if (!driveStick.GetRawButton(ButtonB))
			kick = 0;
		//kicker_motors.left.Set(kick);
		//kicker_motors.right.Set(kick);
	}
	
	void umbrellaMakerAfier()
	{
		if (funcStick.GetRawButton(ButtonX))
			xPressed = true;
		else 
			xPressed = false;
		
		if (xPressed && !funcStick.GetRawButton(ButtonX))
			buttonPresses += 1;
		
		if (buttonPresses % 1 == 0 && !buttonPresses % 2 == 0 && !buttonPresses % 3 == 0 && !buttonPresses % 4 == 0)
			umbrella.Set(DoubleSolenoid::kForward);
		if (buttonPresses % 2 == 0 && !buttonPresses % 4 == 0)
			umbrella.Set(DoubleSolenoid::kOff);
		if (buttonPresses % 3 == 0)
			umbrella.Set(DoubleSolenoid::kReverse);
		if (buttonPresses % 4 == 0)
			umbrella.Set(DoubleSolenoid::kOff);
	}

	void ramp(){
		float pow=driveStick.GetRawAxis(RightY);
		pow=(fabs(pow)>0.8)?pow:0.0;
		DriverStationLCD::GetInstance()->PrintfLine(DriverStationLCD::kUser_Line6, "Ramp=%f", driveStick.GetRawAxis(RightY));
		rampV.Set(pow);
	}
	 
	void roller(){		
		if(funcStick.GetRawButton(ButtonB)){
			if(!buttonBDown){
				buttonBDown = true;
				if(rollerOn){
					rollerOn = false;
					rollerV.Set(0);
				}else{
					rollerOn = true;
					rollerV.Set(RollerMoterPower);
				}
			}
		}else{
			buttonBDown = false;
		}
		if(funcStick.GetRawButton(ButtonA)){
			rampPnumatic1.Set(DoubleSolenoid::kForward);
			rampPnumatic2.Set(DoubleSolenoid::kForward);
		}else if(funcStick.GetRawButton(ButtonY)){
			rampPnumatic1.Set(DoubleSolenoid::kReverse);
			rampPnumatic2.Set(DoubleSolenoid::kReverse);
		}else{
			rampPnumatic1.Set(DoubleSolenoid::kOff);
			rampPnumatic2.Set(DoubleSolenoid::kOff);
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
