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

//#define FRONTRIGHTMOTOR 1
//#define BACKRIGHTMOTOR 2
//#define FRONTLEFTMOTOR 3
//#define BACKLEFTMOTOR 4
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
//#define RIGHTENCODER_A 12
//#define RIGHTENCODER_B 11
//#define LEFTENCODER_A 14
//#define LEFTENCODER_B 13
#define RIGHTENCODER_A 14
#define RIGHTENCODER_B 13
#define LEFTENCODER_A 12
#define LEFTENCODER_B 11
#define Kp 0.010
#define	Ki 0.0000
#define	Kd 0.0
//#define Kp 0.0
//#define	Ki 0.0
//#define	Kd 0.0
#define AUTODRIVECPS 800.0
#define AUTOENDDISTANCE 2600 // less than 9000  // was 2500 originally //3000 at terra hote
#define PIDTOLERANCE 5.0

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
	bool beastMode = false;


}

void DriveControl::initialize() {
	leftEncoder->Reset();
	rightEncoder->Reset();
	leftEncoder->Start();
	rightEncoder->Start();
	leftEncoder->SetDistancePerPulse(REVOLUTIONS);
	rightEncoder->SetDistancePerPulse(REVOLUTIONS);
	pneumaticsControl->shiftUp();
	//	controllerLeft->Disable();
	//	controllerRight->Disable();
	autoTimer.Stop();
	//shiftState = Init;
	//shiftDelay.Reset();
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
	//	controllerLeft->SetInputRange(-10000.0, 10000.0);
	//	controllerRight->SetInputRange(-10000.0, 10000.0);
	//	controllerLeft->SetOutputRange(-10000.0, 10000.0);
	//	controllerRight->SetOutputRange(-10000.0, 10000.0);
	//	controllerLeft->SetPercentTolerance(85);
	//	controllerRight->SetPercentTolerance(85);
	//	controllerRight->SetContinuous();
	//	controllerLeft->SetContinuous();
	//	controllerLeft->Reset();
	//	controllerRight->Reset();
	autoTimer.Stop();
	autoTimer.Reset();
	autoTimer.Start();
	previousAutoTime = autoTimer.Get();
	totalPositionChange = 0.0;
	//	currentAutoState = AutoDrive;
	//
	//	controllerLeft->Enable();
	//	controllerRight->Enable();
	//	controllerLeft->SetSetpoint(0.0);
	//	controllerRight->SetSetpoint(0.0);
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
 * 
 * 
 * 
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

bool DriveControl::autoPIDDrive() {
	double timeChange = (autoTimer.Get() - previousAutoTime);
	previousAutoTime = autoTimer.Get();
	int countLeft = leftEncoder->Get();
	int countRight = rightEncoder->Get();
	//	double countAverage = (countLeft - countRight) / (2.0);
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "AD: %s, RAS:%f",
	//			GetAutoStateString(), autoDriveRampProfile(timeChange));
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "LC%i, RC%i", countLeft,
	//			countRight);
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "LM%f, RM%f",
	//			this->motorBackLeft->Get(), this->motorBackRight->Get());
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "LP%f, RP%f",
	//			controllerLeft->GetSetpoint(), controllerRight->GetSetpoint());
	dsLCD->UpdateLCD();
	switch (currentAutoState) {
	case AutoDrive:
		//		if ((countAverage > AUTOENDDISTANCE - PIDTOLERANCE)) {
		//			controllerLeft->SetSetpoint(AUTOENDDISTANCE);
		//			controllerRight->SetSetpoint(-AUTOENDDISTANCE);
		//			currentAutoState = AutoStopped;
		//		} else {
		float positionChange = autoDriveRampProfile(timeChange);
		float newSetpointLeft = controllerLeft->GetSetpoint() + positionChange;
		float newSetpointRight = controllerRight->GetSetpoint()
				- positionChange;
		if (newSetpointLeft >= AUTOENDDISTANCE) {
			newSetpointLeft = AUTOENDDISTANCE;
		}
		if (newSetpointRight <= -AUTOENDDISTANCE) {
			newSetpointRight = -AUTOENDDISTANCE;
		}
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "SL%f, SR%f",
				newSetpointLeft, newSetpointRight);
		dsLCD->UpdateLCD();

		controllerLeft->SetSetpoint(newSetpointLeft);
		controllerRight->SetSetpoint(newSetpointRight);

		//		}
		break;

	case AutoStopped:
		//controllerLeft->Disable();
		//controllerRight->Disable();
		return true;

	}
	//	this->pIDControlOutputLeft->PIDWrite(0.2);
	//	this->pIDControlOutputRight->PIDWrite(0.2);

	return false;
}
float DriveControl::autoDriveRampProfile(float timeChange) {
	return (timeChange * AUTODRIVECPS);
}
/*
 * Runs Arcade Drive
 */
void DriveControl::runArcadeDrive() {
	// friction value is added as a constant to motor to make it more responsive to joystick at lower value
	float moveValue = INITIAL;
	float rotateValue = INITIAL;
	float frictionValue = INITIAL;
	float rotateFriction = INITIAL;
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

	myRobot->ArcadeDrive(
			((-1.0) * ((moveValue + frictionValue) / SpeedControl)),
			((-1.0) * ((rotateValue + rotateFriction) / SpeedControl)));
	//"Backwards" turning //myRobot.ArcadeDrive(((moveValue + frictionValue) / SpeedControl), ((rotateValue + rotateFriction) / SpeedControl));
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "ELC: %i, ERC: %i",
			leftEncoder->Get(), rightEncoder->Get());
	dsLCD->UpdateLCD();

}

void DriveControl::beastMode() {
	bool isBPressed = xbox->isBPressed();
	/*
	if (isBPressed) {
		beastMode = true;
	} 
	
	if (beastMode) {
		SpeedControl = 0.75;
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "BEAST MODE");
	} else {
		SpeedControl = 1.00;
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "NORMAL MODE");
	}
*/
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
	beastMode();
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
