#include "WPILib.h"
#include "sysLib.h"
#include "taskLib.h"
#include "semLib.h"
//#include "SpeedController.h"
/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */

class MyRobotDrive : public RobotDrive{
    //CANJaguar* jag1, jag2, jag3, jag4;
    Jaguar* jag1;
    Jaguar* jag2;
    Jaguar* jag3;
    Jaguar* jag4;
   
public:
    /* left left right right */
   
    MyRobotDrive(Jaguar* j1,Jaguar* j2,Jaguar* j3,Jaguar* j4):RobotDrive((SpeedController *)NULL,(SpeedController *)NULL)
    {       
        jag1 = j1;
        jag2 = j2;
        jag3 = j3;
        jag4 = j4;
    }
   
    void SetLeftRightMotorOutputs(float leftOutput, float rightOutput){
        jag1->Set(leftOutput);
        jag2->Set(leftOutput);
        jag3->Set(rightOutput);
        jag4->Set(rightOutput);
    }
};
struct ImageData{
    float x;
    float y;
    float radius;
};

class ImageAnalysisCommunication;

static void image_analysis_communication_thread(ImageAnalysisCommunication *cls){
    cls->_thread();
}

#define STACK_SIZE      (8192)
class ImageAnalysisCommunication{
public:
    ImageAnalysisCommunication(const char *address, int port): address(m_address), port(m_port){
        m_task_id=taskSpawn("ImageAnalysisCommunication", 5, VX_FX_TASK, STACK_SIZE, (FUNCPTR)image_analysis_communication_thread, (int)this, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
    virtual ~ImageAnalysisCommunication(){
        taskDelete(m_task_id);
    }
    virtual void _thread(){

    }
private:
    const char m_address;
    int m_port;
    ImageData m_image_data;
    int m_task_id;
};

class RobotDemo : public SimpleRobot
{
    MyRobotDrive* cDrive;
    //RobotDrive myRobot; // robot drive system
    Joystick stick; // only joystick
    Jaguar jag1;
    Jaguar jag2;
    Jaguar jag3;
    Jaguar jag4;
public:
    RobotDemo():       
        stick(1),
        jag1(1),
        jag2(2),
        jag3(3),
        jag4(4)
    {
       
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

    /**
     * Drive left & right motors for 2 seconds then stop
     */
    void Autonomous()
    {
        
    }

    /**
     * Runs the motors with arcade steering.
     */
    void OperatorControl()
    {
    //    myRobot.SetSafetyEnabled(true);
        while (IsOperatorControl())
        {
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