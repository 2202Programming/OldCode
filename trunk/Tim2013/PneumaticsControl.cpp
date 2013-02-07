/*
 * PneumaticsController.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: Beasty
 */

#include "PneumaticsControl.h"

PneumaticsControl::PneumaticsControl() {

	xbox = XboxController::getInstance();
	dsLCD = DriverStationLCD::GetInstance();
	// pressure switch is at Digital input 3,
	// compressor channel is at relay output 1,
	compressor = new Compressor(1, 4);
	//solenoid module is 2,
	//solenoid channel is at port 1 of the module
	triggerSolenoid = new Solenoid(1, 1);
	retractSolenoid = new Solenoid(1, 2);
	firing = false;
}
void PneumaticsControl::initialize() {
	// the solenoid shuts itself off automatically at about 120 psi so we do not have to shut it off for safety reasons.
	compressor->Start();
	triggerSolenoid->Set(false);
	retractSolenoid->Set(true);
}

void PneumaticsControl::initializeAutonomous() {
}
/*
 * when Trigger is pressed, the solenoid switches so that it moves a disk into the shooter wheels
 * otherwise it is back and allows a disk to be loaded.
 */
void PneumaticsControl::fire() {

	if (xbox->isAPressed()) {
		if (triggerSolenoid->Get() == false) {
			triggerSolenoid->Set(true);
			retractSolenoid->Set(false);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "firing");
			solenoidTimer.Reset();
			solenoidTimer.Start();
		}
	}
	if (solenoidTimer.Get() > 1) {
		solenoidTimer.Stop();
		triggerSolenoid->Set(false);
		retractSolenoid->Set(true);
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "retracting");
	}

	dsLCD->UpdateLCD();
}

void PneumaticsControl::run() {

	fire();

}
