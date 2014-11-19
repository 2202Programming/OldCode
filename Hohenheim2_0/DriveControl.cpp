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

#define FRONTRIGHTMOTOR 4
#define BACKRIGHTMOTOR 3
#define FRONTLEFTMOTOR 2
#define BACKLEFTMOTOR 1
#define REVOLUTIONS .05026
#define DRIVECURVE 0.1
#define DRIVESPEED 0.4
#define STOPPEDSPEED 0.0
#define SPEEDCONTROL  1.00 //1.0 // was 1.5 
#define DEADZONE 0.1
#define FRICTION 0.2
#define INITIAL 0.0
#define RIGHTENCODER_A 14
#define RIGHTENCODER_B 13
#define LEFTENCODER_A 12
#define LEFTENCODER_B 11
#define Kp 0.010
#define	Ki 0.0000
#define	Kd 0.0
#define AUTODRIVECPS 800.0
#define AUTOENDDISTANCE 3110 //2970 //2900// less than 9000  // was 2500 originally //3000 at terra hote
#define PIDTOLERANCE 5.0
#define BEASTMODE 0.75

DriveControl::DriveControl() {
	this->motorBackLeft = new Talon(BACKLEFTMOTOR);
	this->motorFrontLeft = new Talon(FRONTLEFTMOTOR);
	this->motorFrontRight = new Talon(FRONTRIGHTMOTOR);
	this->motorBackRight = new Talon(BACKRIGHTMOTOR);
	// on the other robot, the rightEncoder is false! The practice robot sucks... Aaron sucks.... Kamith is awesome!!
	rightEncoder = new Encoder(RIGHTENCODER_A, RIGHTENCODER_B, true,
			Encoder::k2X);
	leftEncoder = new Encoder(LEFTENCODER_A, LEFTENCODER_B, true, Encoder::k2X);
	this->pIDControlOutputLeft = new PIDControlSubClass(motorBackLeft,
			motorFrontLeft);
	//	this->controllerLeft = new PIDController(Kp, Ki, Kd, leftEncoder,
	//			pIDControlOutputLeft);
	this->pIDControlOutputRight = new PIDControlSubClass(motorBackRight,
			motorFrontRight);
	//	this->controllerRight = new PIDController(Kp, Ki, Kd, rightEncoder,
	//			pIDControlOutputRight);
	myRobot = new RobotDrive(motorFrontLeft, motorBackLeft, motorFrontRight,
			motorBackRight);
	xbox = XboxController::getInstance();
	myRobot->SetExpiration(0.1);
	dsLCD = DriverStationLCD::GetInstance();
	pneumaticsControl = PneumaticsControl::getInstance();
	SpeedControl = SPEEDCONTROL;
	beastMode = false;

}

void DriveControl::initialize() {
	leftEncoder->Reset();
	rightEncoder->Reset();
	leftEncoder->Start();
	rightEncoder->Start();
	leftEncoder->SetDistancePerPulse(REVOLUTIONS);
	rightEncoder->SetDistancePerPulse(REVOLUTIONS);
	pneumaticsControl->shiftUp();
	autoTimer.Stop();
	shiftTimer.Stop();
	shiftTimer.Reset();
	stick_Prev_X = 0.0;
	stick_Prev_Y = 0.0;
	currentShiftState = Idle;
}

void DriveControl::initializeAuto() {
	leftEncoder->Reset();
	leftEncoder->SetDistancePerPulse(1);
	leftEncoder->SetPIDSourceParameter(Encoder::kDistance);
	leftEncoder->Start();
	rightEncoder->Reset();
	rightEncoder->SetDistancePerPulse(1);
	rightEncoder->SetPIDSourceParameter(Encoder::kDistance);
	rightEncoder->Start();
	autoTimer.Stop();
	autoTimer.Reset();
	autoTimer.Start();
	previousAutoTime = autoTimer.Get();
	totalPositionChange = 0.0;
}

/*
 * initialize -> encoders 0
 * 
 * in loop0
 * command = rampValue
 * error = setPoint - encoderValue
 * output = kp * error  kp only
 * output = kp * error + ki * integratorValue * error
 * 
 * integratorvalue => intetegratorvalue + error * change in time
 */

bool DriveControl::autoPIDDrive2() {
	float timeChange = (float) (autoTimer.Get() - previousAutoTime);
	previousAutoTime = autoTimer.Get();
	int countLeft = leftEncoder->Get();
	int countRight = rightEncoder->Get();
	totalPositionChange = autoDriveRampProfile(timeChange)
			+ totalPositionChange;
	if (totalPositionChange >= AUTOENDDISTANCE) {
		totalPositionChange = AUTOENDDISTANCE;
	}
	double leftError = totalPositionChange - countLeft;
	double rightError = (-1.0 * totalPositionChange) - countRight;
	double leftOutput = Kp * leftError;
	double rightOutput = Kp * rightError;

	this->pIDControlOutputLeft->PIDWrite(-leftOutput);
	this->pIDControlOutputRight->PIDWrite(-rightOutput);
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "LE %f, RE:%f",
	//			leftError, rightError);
	////	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "LC%i, RC%i", countLeft,
	////			countRight);
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "LM%f, RM%f",
	//			this->motorBackLeft->Get(), this->motorBackRight->Get());
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "LP%f, RP%f",
	//			leftOutput, rightOutput);
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "PC%f",
	//			totalPositionChange);
	dsLCD->UpdateLCD();
	if ((countLeft - countRight) / (2.0) >= AUTOENDDISTANCE) {
		return true;
	}
	return false;
}

bool DriveControl::autoDrive(double autoDriveDistance) {
	double leftDistance = leftEncoder->GetDistance();
	double rightDistance = rightEncoder->GetDistance();
	double DriveCurve = DRIVECURVE;
	if (rightDistance > leftDistance + 20) {
		DriveCurve = DRIVECURVE - 0.1;
	} else if (leftDistance > rightDistance + 20) {
		DriveCurve = DRIVECURVE + 0.1;
	} else {
		DriveCurve = DRIVECURVE;
	}
	double averageDistance = abs((leftDistance + rightDistance) / 2.0);
	if (averageDistance < autoDriveDistance) {
		myRobot->Drive(DRIVESPEED, DriveCurve);
		return false;
	} else {
		myRobot->Drive(STOPPEDSPEED, STOPPEDSPEED);
		return true;
	}
}

char*DriveControl::GetAutoStateString() {

	switch (currentAutoState) {
	case AutoDrive:
		return "AutoDrive";
	case AutoStopped:
		return "AutoStopped";
	default:
		return "Unknown State";

	}
}

float DriveControl::autoDriveRampProfile(float timeChange) {
	return (timeChange * AUTODRIVECPS);
}

void DriveControl::stickLimiter(float stick_X, float stick_Y) {
	float stick_Delta_Max = 0.025;
	if (pneumaticsControl->isHighGear()) {
		stick_Delta_Max = 0.025;
	} else {
		stick_Delta_Max = 0.025;
	}
	float delta_X = stick_X - stick_Prev_X;
	float delta_Y = stick_Y - stick_Prev_Y;
	float stick_Delta = sqrtf(powf(delta_X, 2) + powf(delta_Y, 2));
	if (stick_Delta > stick_Delta_Max) { //Limiter
		float delta_X_Limited = stick_Delta_Max * (delta_X / stick_Delta);
		float delta_Y_Limited = stick_Delta_Max * (delta_Y / stick_Delta);
		stick_X_Cmd = stick_Prev_X + delta_X_Limited;
		stick_Y_Cmd = stick_Prev_Y + delta_Y_Limited;
	} else { //No limiter on
		stick_X_Cmd = stick_X;
		stick_Y_Cmd = stick_Y;
	}
	//assign current to previous
	stick_Prev_X = stick_X_Cmd;
	stick_Prev_Y = stick_Y_Cmd;

}
/*
 * Runs Arcade Drive
 */
void DriveControl::runArcadeDrive() {
	//bottom press is used to shift the robot
	bool LeftBumperHeld = xbox->isLBumperHeld();
	// friction value is added as a constant to motor to make it more responsive to joystick at lower value
	float moveValue = INITIAL;
	float rotateValue = INITIAL;
	float frictionValue = INITIAL;
	float rotateFriction = INITIAL;
	float stick_X;
	float stick_Y;
	//SpeedControl = SPEEDCONTROL;
	moveValue = xbox->getAxisLeftY();
	rotateValue = xbox->getAxisLeftX();

	// if move value is above the dead zone set friction value to .2
	if (moveValue > DEADZONE) {
		frictionValue = FRICTION;
	} else if (moveValue < -DEADZONE) {
		frictionValue = -FRICTION;
	}
	if (rotateValue > DEADZONE) {
		rotateFriction = FRICTION;
	} else if (rotateValue < -DEADZONE) {
		rotateFriction = -FRICTION;
	}
	stick_X = ((-1.0) * ((moveValue + frictionValue) / SpeedControl));
	stick_Y = ((-1.0) * ((rotateValue + rotateFriction) / SpeedControl));
	stickLimiter(stick_X, stick_Y);

	//The Edited Joystick Value, that is divided by sqrt(2)
	//float commandPower = sqrtf((powf(moveValue,2) + powf(rotateValue,2)))/ (sqrtf(2.0));


	//Idle, ShiftStart, ShiftWait, ShiftComplete
	//Shifting Code with a "stall"
	switch (currentShiftState) {
	case Idle:
		myRobot->SetMaxOutput(1.00);
		if (LeftBumperHeld && !pneumaticsControl->isHighGear()) {
			currentShiftState = ShiftWait;
			shiftTimer.Reset();
			shiftTimer.Start();
			myRobot->SetMaxOutput(0.50);
		} else if (!LeftBumperHeld && pneumaticsControl->isHighGear()) {
			if (abs(rightEncoder->GetRate()) < 0.10 && abs(
					leftEncoder->GetRate()) < 0.10) {
				//currentShiftState = ShiftWait;
				//shiftTimer.Reset();
				//shiftTimer.Start();
				//myRobot->SetMaxOutput(0.50);
				pneumaticsControl->shiftDown();

			}
		}
		break;
	case ShiftWait:
		if (shiftTimer.Get() > 0.20) {
			if (pneumaticsControl->isHighGear()) {
				pneumaticsControl->shiftDown();
			} else {
				pneumaticsControl->shiftUp();
			}
			shiftTimer.Stop();
			shiftTimer.Reset();
			shiftTimer.Start();
			currentShiftState = ShiftComplete;
		}
		break;
	case ShiftComplete:
		if (shiftTimer.Get() > .20) {
			myRobot->SetMaxOutput(1.00);
			currentShiftState = Idle;
			shiftTimer.Stop();
			shiftTimer.Reset();
		}
		break;
	}

	myRobot->ArcadeDrive(stick_X_Cmd, stick_Y_Cmd);
	//((-1.0) * ((moveValue + frictionValue) / SpeedControl)),
	//((-1.0) * ((rotateValue + rotateFriction) / SpeedControl)));
	//"Backwards" turning //myRobot.ArcadeDrive(((moveValue + frictionValue) / SpeedControl), ((rotateValue + rotateFriction) / SpeedControl));
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "ELC: %i, ERC: %i",
			leftEncoder->Get(), rightEncoder->Get());
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "RR: %f",
			rightEncoder->GetRate());
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "LR: %f",
			leftEncoder->GetRate());
	dsLCD->UpdateLCD();

}

void DriveControl::BeastMode() { //Makes the acceleration of the robot faster
	float RightJoyStickValue = xbox->getAxisRightY();
	if (RightJoyStickValue >= BEASTMODE) {
		beastMode = true;
	}
	if (RightJoyStickValue <= -BEASTMODE) {
		beastMode = false;
	}

	if (beastMode) {
		SpeedControl = BEASTMODE;
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "BEAST MODE");
	} else {
		SpeedControl = SPEEDCONTROL;
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "NORMAL MODE");
	}
	dsLCD->UpdateLCD();

}

void DriveControl::manualShift() {
	bool LeftBumperHeld = xbox->isLBumperHeld();
	//shifts down if LeftBumper is held
	if (LeftBumperHeld) {
		pneumaticsControl->shiftUp();
	} else {
		pneumaticsControl->shiftDown();
	}

}

void DriveControl::run() {
	runArcadeDrive();
	manualShift();
	//BeastMode();
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
