#include "WPIlib.h"
class Kicker{
        enum state{
            idle,
            goingUp,
            goingDown
        }
        CANJaguar jag;
        Joystick joy;
        const uint32_t BUTTON_PORT = /*port number*/;
        const int POSITION_WHEN_FULLY_UP = /*what it is, need to test*/;
        double stoppedMotorPosition;
    public:
        Kicker();
        void update();
    private:
        void idle;
        void goingUp();
        void goingDown();
};
