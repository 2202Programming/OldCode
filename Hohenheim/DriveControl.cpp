#include "DriveControl.h"
#include "XboxController.h"
#include "Math.h"
#include "PneumaticsControl.h"
#include "WPILib.h"

#define DEFAULT_MIN_POWER .44

//we're not 100% sure about these. I would check but there is no internet. Hey, it works
#define FRONTLEFTMOTOR 3
#define BACKLEFTMOTOR 4
#define FRONTRIGHTMOTOR 1
#define BACKRIGHTMOTOR 2
#define BALLMOTOR 5
#define SHOOTERMOTOR 5
#define AUTOBACKSPEED .8
#define SHIFTLOWSPEED .5
#define SHIFTHIGHSPEED .85
#define REVOLUTIONS .05026
#define FSHIFTUPSPEED 65.0
#define FSHIFTDOWNSPEED 60.0
#define BSHIFTDOWNSPEED 60.0
#define BSHIFTUPSPEED 65.0


DriveControl::DriveControl() :
	myRobot(FRONTLEFTMOTOR, BACKLEFTMOTOR, FRONTRIGHTMOTOR, BACKRIGHTMOTOR){
	xbox = XboxController::getInstance();
	rightEncoder = new Encoder(12, 11, true, Encoder::k2X);
	leftEncoder = new Encoder(14, 13, false, Encoder::k2X);
	myRobot.SetExpiration(0.1);
	dsLCD = DriverStationLCD::GetInstance();
	pneumaticsControl = PneumaticsControl::getInstance();
	MinPower = DEFAULT_MIN_POWER;
	BallGrabber = new Victor(BALLMOTOR);
	RightMotorSpeed = 0;
	Timer waitTime;
	counter = 0;
	LeftMotorSpeed = 0;
	ArcadeRotateSpeed = 0;
	maxValue = 0;
	
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
	float rightMoveValue = 0.0;

	// friction value is added as a constant to motor to make it more responsive to joystick at lower value
	float frictionValue = 0.0;
	float rotateFriction = 0.0;
	float SpeedControl = 1.5;
	moveValue = xbox->getAxisLeftY();
	rotateValue = xbox->getAxisLeftX();
	rightMoveValue = xbox->getAxisRightY();
	
	BallGrabber->Set((rightMoveValue) / SpeedControl);
	
	

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

	
	myRobot.ArcadeDrive(((moveValue + frictionValue) / SpeedControl),
			(-1.0 * (rotateValue + rotateFriction)) / SpeedControl);

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
	
	bool aPress = xbox->isAPressed();
	bool pistonState = pneumaticsControl->isPistonOn();
	//bool bPress = xbox->isBPressed();
	if(aPress){
		if(pistonState){
		pneumaticsControl->pistonOn();
		}
		else if(!pistonState)
		{
		pneumaticsControl->pistonOff();
		}
	}
	
	
	bool leftDirection = leftEncoder->GetDirection();
	bool rightDirection = rightEncoder->GetDirection();
	double leftSpeed = leftEncoder->GetRate();
	double rightSpeed = rightEncoder->GetRate();
	double averageSpeed = (leftSpeed + rightSpeed) / 2.0;
	bool lbPressed = xbox->isLBumperHeld();
	bool rbPressed = xbox->isRBumperHeld();
	
	
	bool isHighGear = pneumaticsControl->isHighGear();

	//manual override
	if (lbPressed) { //shift down if lb is held
		pneumaticsControl->shiftDown();
	} else if (rbPressed) { //shift up if rb is held
		pneumaticsControl->shiftUp();
	}
	
	
	//Auto shifting
	 else {
		if (leftDirection != rightDirection) { //low gear if turning
			pneumaticsControl->shiftDown();
		}
		else if (leftDirection == true && rightDirection == true) { //forward
			if (averageSpeed > FSHIFTUPSPEED && !isHighGear) {
				pneumaticsControl->shiftUp();
			} else if (averageSpeed < FSHIFTDOWNSPEED && isHighGear) {
				pneumaticsControl->shiftDown();
				counter++;
			}
		} else { //backward
			if (abs(averageSpeed) > BSHIFTUPSPEED){
				pneumaticsControl->shiftUp();
			} else if (abs(averageSpeed) < BSHIFTDOWNSPEED) {
				pneumaticsControl->shiftDown();
				//Wait(0.05);
			}
		}
	} 
	
	
	
	bool reset = xbox->isBPressed();
	if(reset){
		maxValue = 0;
	}
	if(maxValue < averageSpeed){
		maxValue = averageSpeed;
	}
	
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "L: %i LD: %f",
			leftEncoder->Get(), leftEncoder->GetDistance());
	
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "R: %i RD: %f",
			rightEncoder->Get(), rightEncoder->GetDistance());
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "LS: %f in/s",
			leftEncoder->GetRate());
//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "RS: %f in/s",
//			rightEncoder->GetRate());
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "MaxValue: %f ",
				maxValue);

	dsLCD->UpdateLCD();

}

bool DriveControl::runAuto() {
	myRobot.ArcadeDrive(AUTOBACKSPEED, AUTOBACKSPEED);
	return (true);
}

