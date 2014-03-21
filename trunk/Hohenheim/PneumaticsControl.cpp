/*
 * PneumaticsController.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: Beasty
 */
#include "PneumaticsControl.h"

//L-Left R-Right S-Shift G(BG)-BallGrabber 

#define RETRACTTIME 1.0
#define SHIFTHIGH 10.0
#define SHIFTLOW 8.0
#define SOLENOIDCOMPONENT 2
#define LS_A 7
#define LS_B 8
#define LG_A 3
#define LG_B 4
#define RS_A 2 
#define RS_B 1
#define RG_A 5
#define RG_B 6
#define BGLIMITSWITCH 4


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
	shiftControlL = new DoubleSolenoid(SOLENOIDCOMPONENT,LS_A , LS_B );
	shiftControlR = new DoubleSolenoid(SOLENOIDCOMPONENT,RS_A , RS_B);
	ballGrabberControlR = new DoubleSolenoid(SOLENOIDCOMPONENT, RG_A, RG_B);
	ballGrabberControlL = new DoubleSolenoid(SOLENOIDCOMPONENT, LG_A, LG_B);
	ballGrabberExtendLimit = new DigitalInput(BGLIMITSWITCH);
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

void PneumaticsControl::compressorEnable(){
	compressor->Start();
}

void PneumaticsControl::compressorDisable(){
	compressor->Stop();
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

bool PneumaticsControl::isHighGear() {
	return highGear;
}

int PneumaticsControl::ReadSwitch(){
return ballGrabberExtendLimit->Get();
	
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

void PneumaticsControl::disable() {
	compressor->Stop();
	shiftControlL->Set(DoubleSolenoid::kOff);
	shiftControlR->Set(DoubleSolenoid::kOff);
	ballGrabberControlR->Set(DoubleSolenoid::kOff);
	ballGrabberControlL->Set(DoubleSolenoid::kOff);
}

