#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#define Pi					3.14159265359 // Pi with good accuracy, because M_PI wouldn't work

#define WheelDiameter		6
#define WheelCircumference	(Pi * WheelDiameter) // Convert diameter to circumference

// Format for naming port numbers: TypeName

// Encoders
#define EncoderTestA		13
#define EncoderTestB		14

//Ramp 
#define RampLimitUp         (1) //Ramp Limit switch for top(+) limit
#define RampLimitDown       (2) //Ramp Limit switch for top(-) limit
#define RampVictor 			(1) //Ramp Victor port

//Roller 
#define RollerVictor		 (4) //Roller Victor port
#define RollerPnumaticA1     (1) //Roller Pnumatic Port
#define RollerPnumaticB1     (2) //Roller Pnumatic Port
#define RollerPnumaticA2     (1) //Roller Pnumatic Port
#define RollerPnumaticB2     (2) //Roller Pnumatic Port
#define RollerMoterPower    (-1) //Roller Moter Power

//Umbrella 
#define UmbrellaSolenoidOn (3)
#define UmbrellaSolenoidOff (4)


#endif
