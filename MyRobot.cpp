#include "WPILib.h"
#include "imageanalysisclient.h"
//#include "SpeedController.h"

class MyRobotDrive : public RobotDrive {
    //CANJaguar* jag1, jag2, jag3, jag4;
    Jaguar* jag1;
    Jaguar* jag2;
    Jaguar* jag3;
    Jaguar* jag4;

public:
    /* left left right right */

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
    MyRobotDrive* cDrive;
    //RobotDrive myRobot; // robot drive system
    Joystick stick; // only joystick
    Jaguar jag1;
    Jaguar jag2;
    Jaguar jag3;
    Jaguar jag4;
    ImageAnalysisClient iaClient;
public:
    RobotDemo():
        stick(1),
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
#define CENTER_LINE         (IDK)
#define CENTER_THRESHOLD    (IDK)
#define RADIUS_TARGET       (IDK)
#define RADIUS_THRESHOLD    (IDK)
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
        while (IsOperatorControl()) {
            cDrive->ArcadeDrive(stick);
            Wait(0.005);                // wait for a motor update time
        }
        delete(cDrive);
    }

    /**
     * Runs during test mode
     */
    void Test() {

    }
};

START_ROBOT_CLASS(RobotDemo);
