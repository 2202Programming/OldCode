#include "DriveControl.h"
#include "XboxController.h"
#include "Math.h"
#include "PneumaticsControl.h"
#include "WPILib.h"

#define DEFAULT_MIN_POWER .44

//Defined Constant. Can Be Changed
#define FRONTRIGHTMOTOR 1
#define BACKRIGHTMOTOR 2
#define FRONTLEFTMOTOR 3
#define BACKLEFTMOTOR 4
#define AUTOBACKSPEED .8
#define SHIFTLOWSPEED .5
#define SHIFTHIGHSPEED .85
#define REVOLUTIONS .05026
#define FSHIFTUPSPEED 65.0
#define FSHIFTDOWNSPEED 25.0
#define BSHIFTDOWNSPEED 5.0
#define BSHIFTUPSPEED 65.0
#define SHIFTDELAYINSECONDS 3.0

DriveControl::DriveControl() :
	myRobot(FRONTLEFTMOTOR, BACKLEFTMOTOR, FRONTRIGHTMOTOR, BACKRIGHTMOTOR) {
	xbox = XboxController::getInstance();
	rightEncoder = new Encoder(12, 11, true, Encoder::k2X);
	leftEncoder = new Encoder(14, 13, false, Encoder::k2X);
	myRobot.SetExpiration(0.1);
	dsLCD = DriverStationLCD::GetInstance();
	pneumaticsControl = PneumaticsControl::getInstance();
	MinPower = DEFAULT_MIN_POWER;
	counter = 0;
	maxValue = 0;
	//UpperShooter = new Talon(7);
	//LowerShooter = new Talon(8);
	shiftState = Init;
}

void DriveControl::initialize() {
	shiftDelay.Reset();
	leftEncoder->Reset();
	rightEncoder->Reset();
	accelTimer.Start();
	leftEncoder->Start();
	rightEncoder->Start();
	leftEncoder->SetDistancePerPulse(REVOLUTIONS);
	rightEncoder->SetDistancePerPulse(REVOLUTIONS);
	shiftState = Init;

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
		if (shiftDelay.Get() >= SHIFTDELAYINSECONDS) {
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
}

void DriveControl::setLowGear() {
	pneumaticsControl->shiftDown();
}
void DriveControl::setHighGear() {
	pneumaticsControl->shiftUp();
}

/*
 * Runs Arcade Drive with AutoShift With Manual Shifting Using LBumper and RBumper
 */
void DriveControl::runArcadeDrive() {

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

	myRobot.ArcadeDrive(((moveValue + frictionValue) / SpeedControl),
			(rotateValue + rotateFriction) / SpeedControl);

	
	bool lbPressed = xbox->isLBumperHeld();
	bool rbPressed = xbox->isRBumperHeld();

	//manual override
	if (lbPressed) { //shift down if lb is held
		pneumaticsControl->shiftDown();
	} else if (rbPressed) { //shift up if rb is held
		pneumaticsControl->shiftUp();
	}

	 
	
	//dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "RightEncode: %f ",
	//		rightSpeed);

	dsLCD->UpdateLCD();

}
void DriveControl::run() {
	Shifter();
	runArcadeDrive();
	}
/*
bool DriveControl::runAuto() {
	myRobot.ArcadeDrive(AUTOBACKSPEED, AUTOBACKSPEED);
	return (true);
}
*/
