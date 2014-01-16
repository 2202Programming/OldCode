#include "DriveControl.h"
#include "XboxController.h"
#include "Math.h"
#include "PneumaticsControl.h"

#define DEFAULT_MIN_POWER .44

//we're not 100% sure about these. I would check but there is no internet. Hey, it works
#define FRONTLEFTMOTOR 3
#define BACKLEFTMOTOR 4
#define FRONTRIGHTMOTOR 1
#define BACKRIGHTMOTOR 2
#define AUTOBACKSPEED .8
#define SHIFTLOWSPEED .5
#define SHIFTHIGHSPEED .85
#define REVOLUTIONS .05026
#define SHIFTUPSPEED 20.0
#define SHIFTDOWNSPEED 15.0
DriveControl::DriveControl() :
	myRobot(FRONTLEFTMOTOR, BACKLEFTMOTOR, FRONTRIGHTMOTOR, BACKRIGHTMOTOR) {
	xbox = XboxController::getInstance();
	rightEncoder = new Encoder(12,11,true);
	leftEncoder = new Encoder(14,13,false);
	myRobot.SetExpiration(0.1);
	dsLCD = DriverStationLCD::GetInstance();
	pneumaticsControl = PneumaticsControl::getInstance();
	MinPower = DEFAULT_MIN_POWER;
	RightMotorSpeed = 0;
	LeftMotorSpeed = 0;
	Timer waitTime;
	precisionDrive = false;
	ArcadeMotorSpeed = 0;
	ArcadeRotateSpeed = 0;
}

void DriveControl::initialize() {
	leftEncoder->Reset();
	rightEncoder->Reset();
	accelTimer.Start();
	leftEncoder->Start();
	rightEncoder->Start();
	leftEncoder->SetDistancePerPulse(REVOLUTIONS);
	rightEncoder->SetDistancePerPulse(REVOLUTIONS);
}

// This method does not use acceleration. But it adds friction value to make motor more responsive at lower move values
void DriveControl::runArcadeNoAcceleration() {

	float moveValue = 0.0;
	float rotateValue = 0.0;

	// friction value is added as a constant to motor to make it more responsive to joystick at lower value
	float frictionValue = 0.0;
	float rotateFriction = 0.0;
	float SpeedControl = 1.5;
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

	// if back button is pressed, set the precision drive mode
	bool isBackPressed = xbox->isBackPressed();
	if (isBackPressed) {
		if (precisionDrive) {
			precisionDrive = false;
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "");

		} else {
			precisionDrive = true;
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "MicroCtrl On");
		}
	}
	if (precisionDrive)
		SpeedControl = 2.5;

	myRobot.ArcadeDrive(0.8 * ((moveValue + frictionValue) / SpeedControl),
			(rotateValue + rotateFriction) / SpeedControl);

	// manual shift, toggle
	/*	bool isRBumper = xbox->isRBumperPressed();
	 if (isRBumper) {
	 if (pneumaticsControl->isHighGear()) {
	 pneumaticsControl->shiftDown();
	 } else {
	 pneumaticsControl->shiftUp();
	 }
	 }
	 */
	//auto shifting

	/*
	 if (waitTime.Get() <= 0.000001) {
		if (moveValue >= SHIFTHIGHSPEED) {
			waitTime.Start();
		}
		if(moveValue <= SHIFTLOWSPEED){
			waitTime.Start();
					
		}
	}else{
		if(waitTime.Get() >= 1.0){
			if(pneumaticsControl->isHighGear()){
				pneumaticsControl->shiftDown();
			}else{
				pneumaticsControl->shiftUp();
			}
			waitTime.Stop();
			waitTime.Reset();
					
		}
	}*/
	double leftspeed = leftEncoder->GetRate();
	double rightspeed = rightEncoder->GetRate();
	double averagespeed = (leftspeed + rightspeed) / 2.0;
	if(averagespeed > SHIFTUPSPEED){
		pneumaticsControl->shiftUp();

	}else if(averagespeed < SHIFTDOWNSPEED){
		pneumaticsControl->shiftDown();

	}
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "L: %i LD: %f", leftEncoder->Get(), leftEncoder->GetDistance());
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "R: %i RD: %f", rightEncoder->Get(),rightEncoder->GetDistance());
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "LS: %f in/s", leftEncoder->GetRate());
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "RS: %f in/s", rightEncoder->GetRate());

	dsLCD->UpdateLCD();

}

bool DriveControl::runAuto() {
	myRobot.ArcadeDrive(AUTOBACKSPEED, AUTOBACKSPEED);
	return (true);
}

