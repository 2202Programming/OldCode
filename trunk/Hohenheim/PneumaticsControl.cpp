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
	shiftControlL = new DoubleSolenoid(2, 7, 8);
	shiftControlR = new DoubleSolenoid(2, 2, 1);
	ballGrabberControlR = new DoubleSolenoid(2, 5, 6);
	ballGrabberControlL = new DoubleSolenoid(2, 3, 4);
	ballGrabberExtendLimit = new DigitalInput(4);
	shiftState = false;
	highGear = false; //true if gear is in high
	isBallGrabberExtended = false; //true if piston is extended

}
void PneumaticsControl::initialize() {
	// the solenoid shuts itself off automatically at about 120 psi so we do not have to shut it off for safety reasons.
	compressor->Start();
	shiftControlL->Set(DoubleSolenoid::kReverse);
	shiftControlR->Set(DoubleSolenoid::kReverse);
	this->ballGrabberRetract();
}

bool PneumaticsControl::ballGrabberIsExtended() {
	return isBallGrabberExtended && (ballGrabberExtendLimit->Get() == 0);
}

void PneumaticsControl::ballGrabberExtend() {
	isBallGrabberExtended = true;
	ballGrabberControlR->Set(DoubleSolenoid::kReverse);
	ballGrabberControlL->Set(DoubleSolenoid::kReverse);
}

void PneumaticsControl::ballGrabberRetract() {
	isBallGrabberExtended = false;
	ballGrabberControlR->Set(DoubleSolenoid::kForward);
	ballGrabberControlL->Set(DoubleSolenoid::kForward);
}

void PneumaticsControl::piston() {
	bool isAPress = xbox->isAPressed();
	if (isAPress) {
		if (isBallGrabberExtended) {
			ballGrabberRetract();
		} else {
			ballGrabberExtend();
		}
	}
}

bool PneumaticsControl::isHighGear() {
	return highGear;
}

void PneumaticsControl::shift() {
	bool isRBumper = xbox->isRBumperPressed();
	int compSwitch = compressor->GetPressureSwitchValue();
	if (isRBumper) {
		if (highGear) {
			highGear = false;
			shiftControlL->Set(DoubleSolenoid::kForward);
			shiftControlR->Set(DoubleSolenoid::kForward);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Low Gear");
			dsLCD->UpdateLCD();
		} else {
			highGear = true;
			shiftControlL->Set(DoubleSolenoid::kReverse);
			shiftControlR->Set(DoubleSolenoid::kReverse);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "High Gear");
			dsLCD->UpdateLCD();
		}

	}
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "High Gear: %i",
			compSwitch);
	dsLCD->UpdateLCD();

}

void PneumaticsControl::shiftUp() {
	highGear = true;
	shiftControlL->Set(DoubleSolenoid::kForward);
	shiftControlR->Set(DoubleSolenoid::kForward);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "High Gear");
	dsLCD->UpdateLCD();
}

void PneumaticsControl::shiftDown() {
	highGear = false;
	shiftControlL->Set(DoubleSolenoid::kReverse);
	shiftControlR->Set(DoubleSolenoid::kReverse);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Low Gear");
	dsLCD->UpdateLCD();
}

bool PneumaticsControl::CompressorFull() {
	if ((compressor->GetPressureSwitchValue()) > 0) {
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "compressor on");
		dsLCD->UpdateLCD();
		return true;
	}
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "compressor off");
	dsLCD->UpdateLCD();
	return false;
}

void PneumaticsControl::run() {
	shift();
	//piston();

}
void PneumaticsControl::disable() {
	compressor->Stop();
	shiftControlL->Set(DoubleSolenoid::kOff);
	shiftControlR->Set(DoubleSolenoid::kOff);
	ballGrabberControlR->Set(DoubleSolenoid::kOff);
	ballGrabberControlL->Set(DoubleSolenoid::kOff);
}

