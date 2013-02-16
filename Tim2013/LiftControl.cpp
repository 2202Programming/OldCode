/* This is the CPP file for Lift Control 
 * Current Version:
 * B Button hold to lift. No reverse.
 * */

#include "LiftControl.h"
#include "XboxController.h"

#define LIFTMOTORRELAYPORT 1

static LiftControl *liftcontrol = NULL;
LiftControl *LiftControl::getInstance() {
	if (liftcontrol == NULL) {
		liftcontrol = new LiftControl();
	}
	return liftcontrol;
}

LiftControl::LiftControl() {
	isGoingUp = true;
	xbox = XboxController::getInstance();
	dsLCD = DriverStationLCD::GetInstance();
	solenoidTop = new Solenoid(SOLENOIDMODULE, TOPCHANNEL);
	solenoidBot = new Solenoid(SOLENOIDMODULE, BOTCHANNEL);
}

void LiftControl::initialize() {
	solenoidTop->Set(isGoingUp);
	solenoidBot->Set(!isGoingUp);
}

void LiftControl::initializeAutonomous() {
}

/*
 * If B is pressed for the first time, the solenoids will go into the up position. 
 * Then the bool isGoingUp will be reversed, so the next time B is pressed, 
 * the solenoids will go into the down position. 
 * Cool.
 */
void LiftControl::run() {
	bool isBPressed = xbox->isBPressed();
	if (isBPressed) {
		solenoidTop->Set(!isGoingUp);
		solenoidBot->Set(isGoingUp);
		isGoingUp = !isGoingUp;
	} else {
	}

}
void LiftControl::runAutonomous() {
}

