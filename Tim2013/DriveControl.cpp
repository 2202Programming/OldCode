#include "DriveControl.h"
#include "XboxController.h"
#include "Math.h"

#define DEFAULT_MIN_POWER .44

//we're not 100% sure about these. I would check but there is no internet. Hey, it works
#define FRONTLEFTMOTOR 3
#define BACKLEFTMOTOR 4
#define FRONTRIGHTMOTOR 1
#define BACKRIGHTMOTOR 2

DriveControl::DriveControl() :
	myRobot(FRONTLEFTMOTOR, BACKLEFTMOTOR, FRONTRIGHTMOTOR, BACKRIGHTMOTOR) {
	xbox = XboxController::getInstance();
	myRobot.SetExpiration(0.1);
	dsLCD = DriverStationLCD::GetInstance();
	MinPower = DEFAULT_MIN_POWER;
	RightMotorSpeed = 0;
	LeftMotorSpeed = 0;
}

void DriveControl::initialize() {
	//xbox = XboxController::getInstance();
	myRobot.SetSafetyEnabled(true);
}
void DriveControl::initializeAutonomous() {
	myRobot.SetSafetyEnabled(false);
}
//min value = .44
// max value = 1.0
// original value if at full throttle = 1.0
float DriveControl::scaledOffset(float originalValue, float minValue,
		float maxValue) {

	if (fabs(originalValue) <= 0.1)
		return 0.0;

	if (minValue > maxValue)
		minValue = maxValue;

	if (fabs(originalValue) > 1.0) {
		originalValue = (originalValue > 0) ? 1.0 : -1.0;
	}

	if (originalValue > 0.0) {
		return (maxValue - minValue) * originalValue + minValue;// (1 - .44)*1 + .44 = 1
	} else {
		return (maxValue - minValue) * originalValue - minValue;
	}
}

float DriveControl::scaledOffset(float originalValue, float minValue) {
	return scaledOffset(originalValue, minValue, 1.0);
}
float DriveControl::scaleValue(float originalValue, float offset) {
	if (offset != 1 && originalValue != 0) {
		if (originalValue > 0) {
			return (1.0 / (1.0 - offset)) * originalValue - (offset / (1.0
					- offset));
		} else {
			return (1.0 / (1.0 - offset)) * originalValue + (offset / (1.0
					- offset));
		}
	}
	return 0;
}
float DriveControl::accelerateMotor(float stickValue, float MotorValue) {
	// check for dead zone
	if (stickValue < .1 && stickValue > -.1) {
		stickValue = 0;
	}
	// check if we need to decelerate
	if (stickValue < MotorValue) {
		MotorValue -= .005;
		// checks if Motor decelerated beyond stickValue then set it back to stick value
		if (MotorValue < stickValue) {
			MotorValue = stickValue;
		}
		// else accelerate
	} else {
		MotorValue += .005;
		//checks if Motor accelerated beyond stickValue then set it back to stick value
		if (MotorValue > stickValue) {
			MotorValue = stickValue;
		}

	}

	return MotorValue;

}

void DriveControl::run() {
	float lefty = 0.0;
	float righty = 0.0;
	lefty = xbox->getAxisLeftY();
	righty = xbox->getAxisRightY(); //this is for tank
	//		myRobot.TankDrive(scaledOffset(lefty, MinPower),
	//				scaledOffset(righty, MinPower)); // drive with tank style

	// match motor speed if stick values are only slightly off to drive straight
	if (fabs(lefty - righty) < .05) {
		lefty = righty;
	}
	LeftMotorSpeed = accelerateMotor(lefty, LeftMotorSpeed);
	RightMotorSpeed = accelerateMotor(righty, RightMotorSpeed);

	myRobot.TankDrive(LeftMotorSpeed, RightMotorSpeed);
	//lefty = xbox->getAxisLeftY(); // this is for arcade
	//righty = xbox->getAxisLeftX();
	//myRobot.ArcadeDrive(-scaledOffset(lefty, MinPower),
	//	-scaledOffset(righty, MinPower)); // drive with tank style
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Left: %f",
	//			xbox->getAxisLeftY());
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "right: %f",
	//			xbox->getAxisRightY());
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "scaled left: %f", lefty);
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "scaled rt: %f", righty);
	//	dsLCD->UpdateLCD();
}

bool DriveControl::runAuto() {
	return (true);
}

