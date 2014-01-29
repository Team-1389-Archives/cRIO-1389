#include "kicker.h"

Kicker::Kicker():jag(1, /*MODE*/), state(0), joy(/*port*/){
    jag.setPID(0.5, 0.2, 0.0);//find actual necessary PID values
    jag.configEncoderCodesPerRev(/*whatever the number is*/);
    jag.setSpeedReference(CanJaguar::kSpeedRef_WHATEVER_IT_IS);
    jag.setPositionReference(CanJaguar::kSpeedRef_whichever_one);
    jag.changeControlMode(CanJaguar::kPercentVbus);
    jag.enableControl();
    state = idle;
    stoppedMotorPosition = jag.getPosition();
        }

Kicker::update(){
    switch(state){
        case idle:
            idle();
            break;

        case goingUp:
            goingUp();
            break;

        case goingDown:
            goingDown();
            break;
    }
}

Kicker::idle(){
    if(joystick.getRawButton(BUTTON_PORT)){
        jag.set(1);
        stoppedMotorPosition = jag.getPosition();
        state = goingUp;
    }
}

Kicker::goingUp(){
    if (jag.getPosition() >= POSITION_WHEN_FULLY_UP){
        jag.set(-1);
        state = goingDown;
    }
}

Kicker::goingDown(){
    if (/*fully down, find out how this is determined*/){
        jag.set(0);
        state = idle;
    }
}
