/*
 * PneumaticsController.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: Beasty
 */
#include "PneumaticsControl.h"

#define RETRACTTIME 1.0
#define SHIFTHIGH 10.0
#define SHIFTLOW 8.0

static PneumaticsControl *pneumaticsControl = NULL;
PneumaticsControl *PneumaticsControl::getInstance() {
	if (pneumaticsControl == NULL) {
		pneumaticsControl = new PneumaticsControl();
	}
	return pneumaticsControl;
}

PneumaticsControl::PneumaticsControl() {

	xbox = XboxController::getInstance();
	dsLCD = DriverStationLCD::GetInstance();
	compressor = new Compressor(5, 4);
	rightTrigger = new Solenoid(1,1);
	rightRetract = new Solenoid(1,2);
	leftTrigger = new Solenoid(1,8);
	leftRetract = new Solenoid(1,7);
	pistonR = new Solenoid(1,5);
	retractPistonR = new Solenoid(1,6);
	pistonL = new Solenoid(1,3);
	retractPistonL = new Solenoid(1,4);
	
	shiftState = false;
	highGear = false; //true if gear is in high
	pistonState = false; //true if piston is extended
	
}
void PneumaticsControl::initialize() {
	// the solenoid shuts itself off automatically at about 120 psi so we do not have to shut it off for safety reasons.
	compressor->Start();
	rightTrigger->Set(false);
	rightRetract->Set(true);
	leftTrigger->Set(false);
	leftRetract->Set(true);
	pistonR->Set(false);
	pistonL->Set(false);
	retractPistonR->Set(true);
	retractPistonL->Set(true);
	
	//leftTrigger->Set(false);
	//leftRetract->Set(true);
}



bool PneumaticsControl::isPistonOn() {
	return pistonState;
}

void PneumaticsControl::pistonOn() {
	pistonState = true;
	pistonR->Set(true);
	pistonL->Set(true);
	retractPistonR->Set(false);
	retractPistonL->Set(false);
}

void PneumaticsControl::pistonOff() {
	pistonState = false;
	pistonR->Set(false);
	pistonL->Set(false);
	retractPistonR->Set(true);
	retractPistonL->Set(true);
}

void PneumaticsControl::piston() {
	bool isAPress = xbox->isAPressed();
	if (isAPress) {
		if (pistonState) {
			pistonOff();
		} else {
			pistonOn();
		}
	}
}



bool PneumaticsControl::isHighGear() {
	return highGear;
}

void PneumaticsControl::shift() {
	bool isRBumper = xbox->isRBumperPressed();
	if (isRBumper) {
		if (highGear) {
			highGear = false;
			rightTrigger->Set(true);
			leftTrigger->Set(true);
			rightRetract->Set(false);
			leftRetract-> Set(false);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Low Gear");
			dsLCD->UpdateLCD();

		} else {
			highGear = true;
			rightTrigger->Set(false);
			leftTrigger->Set(false);
			rightRetract->Set(true);
			leftRetract-> Set(true);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "High Gear");
			dsLCD->UpdateLCD();
		}

	}

}

void PneumaticsControl::shiftUp() {
	highGear = true;
	rightTrigger->Set(true);
	leftTrigger->Set(true);
	rightRetract->Set(false);
	leftRetract-> Set(false);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "High Gear");
	dsLCD->UpdateLCD();
}

void PneumaticsControl::shiftDown() {
	highGear = false;
	rightTrigger->Set(false);
	leftTrigger->Set(false);
	rightRetract->Set(true);
	leftRetract-> Set(true);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Low Gear");
	dsLCD->UpdateLCD();
}

bool PneumaticsControl::CompressorFull() {
	if ((compressor->GetPressureSwitchValue()) > 0) {
		return true;
	}
	return false;
}

void PneumaticsControl::run() {
	shift();
	piston();

}