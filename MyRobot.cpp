/*

 The robot code for team 1389 in the 2014 FIRST Robotics Competition. (Aerial Assist)
 The objective is to put large medicine balls into goals after passing them between robots
 No eng game this year


 */

//// HEAD - Includes and Defines

/// Includes
#include "WPILib.h" // The WPI Library
#include <sysLib.h>	// Stuff
#include <semLib.h> // Semaphores
#include <math.h>	// Maths




/// Defines
#define Pi					(3.1415926535898) // Pi to 13 places
#define Tao					(2*Pi) // Tao = 2 Pi
#define Octopi				(8*Pi) // Or is it octopodes?

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

// Motor ports
#define M_DriveLF			(7) // Left, Front Drive motor
#define M_DriveLR			(9) // Left, Front Drive motor
#define M_DriveRF			(8) // Left, Front Drive motor
#define M_DriveRR			(10) // Left, Front Drive motor
// Solenoid Ports

// Sensor ports

// Connection defines
#define Port				(433) // Port to connect to driverstation through TCP
#define MaxLength			(1024) // Max length of BallPosition data size


// PID Values
// PID is used to smoothly use encoders, and is MANDATORY! (Will not work without values)
#define JagP				(1.505) // P Value for Jags (From 2012 manual testing)
#define JagI				(0.003) // I Value for Jags (From 2012 manual testing)
#define JagD				(0.000) // D Value for Jags (From 2012 manual testing)


// Other defines
#define deadzone			(0.07)

/// Version information
#define VersionAuthor		("Tara")
#define VersionNumber		("4.0.0.t.0")  
// Verison number system is A.B.C.D.E
// A = Competition Year Final Digit
// B = Tournament number (Build/Pre season = 0, first competition = 1, second = 2, third = 3, etc.)
// C = Major increment number (Increment after major changes / code merges)
// D = Author Definition letter (First letter of first name / first x+1 where x letters are shared between authors)
// E = Version ID number (Increment every code change, hex numbers preferred)
#define VersionMessage		("TCP Test 1")


struct BallPosition{
	
	float a, b, c;
	
};

//// PRE-BODY - Static Variables and Static Functions

/// Static primative variables


BallPosition myBall;
SEM_ID mutex;

// Motors for all threads

// Drive Motors

// Spikes


// Sensors for all threads


// Controllers
static Joystick *LeftXBOXController, *RightXBOXController; // XBox Controllers

// Driver Station
static DriverStation *ds;

// Driver station LCD display lines, to save time.
static const DriverStationLCD::Line Line1 = DriverStationLCD::kUser_Line1;
static const DriverStationLCD::Line Line2 = DriverStationLCD::kUser_Line2;
static const DriverStationLCD::Line Line3 = DriverStationLCD::kUser_Line3;
static const DriverStationLCD::Line Line4 = DriverStationLCD::kUser_Line4;
static const DriverStationLCD::Line Line5 = DriverStationLCD::kUser_Line5;
static const DriverStationLCD::Line Line6 = DriverStationLCD::kUser_Line6;

/// Classes

// Custom RobotDrive 
//*
class CustomRobotDrive: public RobotDrive {
public:

	float c1, c2;
	float speed;
	CustomRobotDrive() :
		RobotDrive((UINT32) 1, (UINT32) 3) {
		InitRobotDrive();
		speed = 1;
		m_safetyHelper->SetExpiration(48 * 60 * 60); // Two Days
	}
	virtual ~CustomRobotDrive() {
	}
	
	// Override automatic motor setting
	virtual void SetLeftRightMotorOutputs(float leftOutput, float rightOutput) {
		c1 = leftOutput * speed;
		c2 = rightOutput * speed;
		m_safetyHelper->Feed();
	}
	
	
};//*/


// One solenoid device is two solenoid objects. This combines them into one object
class SuperSolenoid {

	Solenoid *open, *close;
	bool isOpen;

public:

	SuperSolenoid(int openPort, int closePort) {
		open = new Solenoid(1, openPort);
		close = new Solenoid(1, closePort);
		isOpen = false;
	}

	void setOpen() {
		isOpen = true;
		open->Set(true);
		close->Set(false);
	}
	void setClose() {
		isOpen = false;
		open->Set(false);
		close->Set(true);
	}
	void Set(bool newState) {
		if (newState)
			setOpen();
		if (!newState)
			setClose();
	}

	bool isOpened() {
		return isOpen;
	}

};



//// LOW-BODY - Alternative Threads and their methods

/// Static Functions


// Absolute Value Function for decimals
static double absolute(double a) {
	if (a < 0)
		return -a;
	return a;
}

// Easy deadzone calculation for joystick axis inputs
static double deadZone(double value, double maxDead) {
	if (absolute(value) < maxDead)
		return 0;
	return value;

}

// Mutex locked functions to copy myBall or set myBall
static BallPosition ballClone(){
	semTake(mutex, -1);
	
	BallPosition b;
	b.a=myBall.a;
	b.b=myBall.b;
	b.c=myBall.c;
	
	semGive(mutex);
	
	return b;
	
}

static void ballSet(BallPosition a){
	semTake(mutex, -1);
	
	myBall.a=a.a;
	myBall.b=a.b;
	myBall.c=a.c;
	
	semGive(mutex);
	
}


/// Other Threads





// Connection thread
static void connectThread(){ // Turns out connect() already exists so we can't use that name
	
	int mySocket= ERROR;
	struct sockaddr_in dest; 
	float hundred=100;

	
	while(true){
		
		DriverStationLCD::GetInstance()->PrintfLine(Line1, "Establishing Connection...");
		DriverStationLCD::GetInstance()->UpdateLCD();
		
		mySocket= socket(AF_INET, SOCK_STREAM, 0);
		
		// Parse IP into an address number
		dest.sin_addr.s_addr = inet_addr(const_cast<char*> ("10.13.89.4"));
		
		 
		memset(&dest, 0, sizeof(dest));                /* zero the struct */
		dest.sin_family = AF_INET;
		dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK); /* set destination IP number - localhost, 127.0.0.1*/ 
		dest.sin_port = htons(Port);                /* set destination port number */
		
		// Connect!
		connect(mySocket, (struct sockaddr *) &dest, sizeof(struct sockaddr));
		
		char data[MaxLength]; // Data array of arbitrary length
		
		int attempts=0;
		
		while(attempts<3){ 
			attempts=0;
			
			while(write(mySocket, "ask\n", 4)==ERROR && attempts<3){
				attempts++;
			}
			
			if(attempts<3) // Reset attempts counter if it has not reached 3 failed attempts
				attempts=0;
			
			
			// read data back
			int trueLength=ERROR;
			while(trueLength==ERROR && attempts<3){
				trueLength = read(mySocket, data, MaxLength);
				attempts++;
			}
			
			/*
			BallPosition temp;
			// parse data into a BallPosition
			ballSet(temp); //*/
			
			taskDelay(CLOCKS_PER_SEC/hundred); // TODO ask marc if 20 ticks is a good wait
			
		}
		
		DriverStationLCD::GetInstance()->PrintfLine(Line1, "Connection lost!");
		DriverStationLCD::GetInstance()->UpdateLCD();
		
		taskDelay(CLOCKS_PER_SEC/hundred);
		
	}
	
}






//// BODY - Main thread and methods


/**	List of motor controllers, solenoids, sensors, and other mechanical interactions:
 * 	
 * 	Drive: 
 * 			???
 * 			
 * 	Shoot:
 * 			???
 * 			
 *	Block:
 *			???
 * 			
 */



class MyRobot: public SimpleRobot {

	// Main thread variables

	int timesRun; // To display how many times code has been run between restarts
	
	// Robot Drive
	CustomRobotDrive *rd;

	//Victors
	
	
	// Jaguars
	
	
	// CANJaguars
	
	
	// Solenoids
	
	
	// Spike (Class is just called Relay)
	
	
	// Sensors
	
	
	// Driver Station
	DriverStationLCD *ds_lcd;
	

	

public:
	MyRobot(void) {
		// Variables *must* be declared in order. The code is VERY picky about this
		
		// Declare all multithread (static) variables in order
		//myBall=new BallPosition;
		mutex = semMCreate(SEM_Q_FIFO);
		
		LeftXBOXController = new Joystick(1);
		RightXBOXController = new Joystick(2);
		ds = DriverStation::GetInstance();

		// Initialize drive motors


		// Declare all main thread variables and motors in order

		timesRun = 0;
		
		rd = new CustomRobotDrive();
		

		//Victors
		
		
		// Jaguars
		
		
		// CANJaguars
		
		
		/* Encoder-controlled motor controller example 
		
		Shooter = new CANJaguar(M_Shooter, CANJaguar::kSpeed);
		
		Shooter->SetPID(JagP, JagI, JagD);
		
		Shooter->ConfigEncoderCodesPerRev(400);
		
		Shooter->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
		
		Shooter->EnableControl(); //*/
		
		// Solenoids
		// Spike Relays
			//compress = new Relay(1, Relay::kForwardOnly);
		
		// Sensors
		
		// Driver station
		ds_lcd = DriverStationLCD::GetInstance();
		
		
		
		// Spawn other threads
		/* EX:
		 taskSpawn("ThreadName",Task::kDefaultPriority,VX_FP_TASK,2000,(FUNCPTR)(MethodName),0,0,0,0,0,0,0,0,0,0);
		 */
		taskSpawn("Connect", Task::kDefaultPriority, VX_FP_TASK, 2000,(FUNCPTR) connectThread, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		

		// Default display screen
		startupDisplay();

	}
	void startupDisplay() {

		ds_lcd->Clear();
		ds_lcd->Printf(Line1, 1, "Robot booted!");
		ds_lcd->Printf(Line2, 1, "Version: %s", VersionNumber);
		ds_lcd->Printf(Line3, 1, "Version by %s", VersionAuthor);
		ds_lcd->Printf(Line4, 1, VersionMessage);
		ds_lcd->UpdateLCD();
	}
	

	void postUseDisplay() {
		ds_lcd->Clear();
		ds_lcd->Printf(Line1, 1, "Robot ready!");
		ds_lcd->Printf(Line2, 1, "Version %s", VersionNumber);
		ds_lcd->Printf(Line3, 1, "Version by %s", VersionAuthor);
		ds_lcd->Printf(Line4, 1, VersionMessage);
		ds_lcd->Printf(Line5, 1, "(Ran %d times)", timesRun);
		ds_lcd->UpdateLCD();
	}


	void Autonomous(void) {
		timesRun++;

		// Autonomous code here
		
		while (IsAutonomous() && IsEnabled()) {
			
		}
		
		// Turn off any motors that might be left on

		postUseDisplay();

	}

	void OperatorControl(void) {
		timesRun++;

		
		while (IsEnabled() && IsOperatorControl()) {

			// Print
			ds_lcd->Clear();
			ds_lcd->Printf(Line1, 1, "Engaging Manual Control~");

			
			// Methods
			

			ds_lcd->UpdateLCD(); // Update driver station once priority is returned from other threads
			// This guarentees other threads can update the display
		} // Code from here down in OperatorControl will not be reached at the tournament (I think)

		
		// Disable all motors that might be in use

		postUseDisplay();
	}

	/* 2013 drive code
	void driveTest() {
		
		rd->speed = LeftXBOXController->GetRawButton(ButtonX) ? 1.0 : 0.5;
		if (LeftXBOXController->GetRawButton(ButtonY))
			rd->speed = 0.25;

		rd->ArcadeDrive(LeftXBOXController, false);

		DriveLF->Set(-rd->c1);
		DriveLR->Set(rd->c1);
		DriveRF->Set(rd->c2);
		DriveRR->Set(rd->c2);

	}//*/
	
	
	/* 2011 holonomic drive (mechanum)
	void holoDrive(){

	    float LeftX1; // This will gain the X value of left joystick on the 360 
	    float LeftY1; // This will gain the Y value of the left joystick on the 360 
	    float RightX1;
	    float RightY1; //This will gain the Y value of the right joystick on the 360 
	    float Mag; //This will get the magnitude for Holonomic Drive 
	    float Direct; //Get direction of Holonomic Drive 
	    float Rotate; //Just pass Right X into it for the rotation 

		
		
		LeftX1= LeftXBOXController->GetRawAxis(LeftX); 
		if (absolute(LeftX1) < .2) { 
		    LeftX1 = 0; 
		} 
		    
		LeftY1= LeftXBOXController->GetRawAxis(LeftY); 
		if (absolute(LeftY1) < .2) { 
			LeftY1 = 0; 
		} 
		RightX1= LeftXBOXController->GetRawAxis(RightX); 
		if (absolute(RightX1) < .3) { 
		    RightX1 = 0; 
		} 
		RightY1= LeftXBOXController->GetRawAxis(RightY); 
		if (absolute(RightY1) < .15) { 
			RightY1 = 0; 
		} 
		Rotate=0; 
		Mag= sqrt((LeftY1*LeftY1)+(LeftX1*LeftX1)); 
		if (LeftX1==0) { //Do not change. Keeps from dividing by zero 
			if (LeftY1>=0) { 
					Direct = 180; 
			} else { 
				Direct = 0; 
			}
		} else { 
			Direct = (atan2(LeftX1, (-1)*LeftY1)*(180/3.14159)); //Calculates direction in degrees in X/-Y 
		} 
		Rotate = RightX1; 
		myRobot->HolonomicDrive(Mag, Direct, Rotate); 
		
	}//*/
	

	
	
};

START_ROBOT_CLASS(MyRobot)
;

// Heatdeath of the universe, and snacks
