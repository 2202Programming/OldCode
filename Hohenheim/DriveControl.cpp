#include "DriveControl.h"
#include "XboxController.h"
#include "Math.h"
#include "PneumaticsControl.h"
#include "WPILib.h"

/*
 #define AUTOBACKSPEED .8
 #define SHIFTLOWSPEED .5
 #define SHIFTHIGHSPEED .85
 #define FSHIFTUPSPEED 60.0
 #define FSHIFTDOWNSPEED 50.0
 #define BSHIFTDOWNSPEED 50.0
 #define BSHIFTUPSPEED 60.0
 #define SHIFTDELAYINSECONDS 1.0
 */

#define FRONTRIGHTMOTOR 1
#define BACKRIGHTMOTOR 2
#define FRONTLEFTMOTOR 3
#define BACKLEFTMOTOR 4
#define REVOLUTIONS .05026
#define DRIVECURVE 0.1
#define DRIVESPEED 0.4
#define STOPPEDSPEED 0.0
#define SPEEDCONTROL 1.5
#define RIGHTENCODER_A 12
#define RIGHTENCODER_B 11
#define LEFTENCODER_A 14
#define LEFTENCODER_B 13

DriveControl::DriveControl() :
	myRobot(FRONTLEFTMOTOR, BACKLEFTMOTOR, FRONTRIGHTMOTOR, BACKRIGHTMOTOR) {
	xbox = XboxController::getInstance();
	rightEncoder = new Encoder(RIGHTENCODER_A, RIGHTENCODER_B, true,
			Encoder::k2X);
	leftEncoder
			= new Encoder(LEFTENCODER_A, LEFTENCODER_B, false, Encoder::k2X);
	myRobot.SetExpiration(0.1);
	dsLCD = DriverStationLCD::GetInstance();
	pneumaticsControl = PneumaticsControl::getInstance();
}

void DriveControl::initialize() {
	leftEncoder->Reset();
	rightEncoder->Reset();
	leftEncoder->Start();
	rightEncoder->Start();
	leftEncoder->SetDistancePerPulse(REVOLUTIONS);
	rightEncoder->SetDistancePerPulse(REVOLUTIONS);
	pneumaticsControl->shiftUp();
	//shiftState = Init;
	//shiftDelay.Reset();
}

bool DriveControl::autoDrive(double autoDriveDistance) {
	double leftDistance = leftEncoder->GetDistance();
	double rightDistance = rightEncoder->GetDistance();
	double averageDistance = abs((leftDistance + rightDistance) / 2.0);
	if (averageDistance < autoDriveDistance) {
		myRobot.Drive(DRIVESPEED, DRIVECURVE);
		return false;
	} else {
		myRobot.Drive(STOPPEDSPEED, STOPPEDSPEED);
		return true;
	}
}

/*
 * Runs Arcade Drive with AutoShift With Manual Shifting Using LBumper and RBumper
 */
void DriveControl::runArcadeDrive() {
	// friction value is added as a constant to motor to make it more responsive to joystick at lower value
	float moveValue = 0.0;
	float rotateValue = 0.0;
	float frictionValue = 0.0;
	float rotateFriction = 0.0;
	float SpeedControl = SPEEDCONTROL;
	moveValue = xbox->getAxisLeftY();
	rotateValue = xbox->getAxisLeftX();

	// if move value is above the dead zone set friction value to .2
	if (moveValue > .1) {
		frictionValue = 0.2;
	} else if (moveValue < -.1) {
		frictionValue = -.2;
	}
	if (rotateValue > .1) {
		rotateFriction = 0.2;
	} else if (rotateValue < -.1) {
		rotateFriction = -.2;
	}

	myRobot.ArcadeDrive(((moveValue + frictionValue) / SpeedControl),
			((-1.0) * ((rotateValue + rotateFriction) / SpeedControl)));
//	myRobot.ArcadeDrive(((moveValue + frictionValue) / SpeedControl), ((rotateValue + rotateFriction) / SpeedControl));
	dsLCD->UpdateLCD();
}

void DriveControl::manualShift() {
	bool LeftBumperHeld = xbox->isLBumperHeld();
	//shifts down if LeftBumper is held
	if (LeftBumperHeld) {
		pneumaticsControl->shiftDown();
	} else {
		pneumaticsControl->shiftUp();
	}

}

void DriveControl::run() {
	runArcadeDrive();
	manualShift();
}

/*
 bool DriveControl::runAuto() {
 myRobot.ArcadeDrive(AUTOBACKSPEED, AUTOBACKSPEED);
 return (true);
 }

 void DriveControl::Shifter() {
 double leftSpeed = leftEncoder->GetRate();
 double rightSpeed = rightEncoder->GetRate();
 double averageSpeed = abs((leftSpeed + rightSpeed) / 2.0);
 switch (shiftState) {
 case Init:
 setLowGear();
 shiftState = Low;
 break;
 case Low:
 if (averageSpeed >= FSHIFTUPSPEED) {
 setHighGear();
 shiftDelay.Reset();
 shiftDelay.Start();
 shiftState = DelayToHigh;
 }
 break;
 case High:
 if (averageSpeed <= FSHIFTDOWNSPEED) {
 setLowGear();
 shiftDelay.Reset();
 shiftDelay.Start();
 shiftState = DelayToLow;
 }
 break;
 case DelayToLow:
 if (Delay.Get() >= SHIFTDELAYINSECONDS) {
 shiftState = Low;
 shiftDelay.Stop();

 }
 break;
 case DelayToHigh:
 if (shiftDelay.Get() >= SHIFTDELAYINSECONDS) {
 shiftState = High;
 shiftDelay.Stop();
 }
 break;
 }
 
 dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "Drive Encoder x: %f", leftSpeed);
 dsLCD->UpdateLCD();
 }



 void DriveControl::setLowGear() {
 pneumaticsControl->shiftDown();
 }
 void DriveControl::setHighGear() {
 pneumaticsControl->shiftUp();
 }
 * 
 */
