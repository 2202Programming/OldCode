#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H

#include "WPILib.h"
#include <cmath>
#include "XboxController.h"

class DriveControl {
public:
	DriveControl();
	void initialize();
	void initializeAutonomous();
	void runTank();
	void runArcade();
	void runArcadeNoAcceleration();	
	bool runAuto();

private:
	//float scaledOffset(float originalValue, float minValue);
	//float scaledOffset(float originalValue, float minValue, float maxValue);
	//float scaleValue(float originalValue, float offset);
	float accelerateMotor(float stickValue, float MotorValue, float loopTime);
	float accelerateTurnMotor(float stickValue, float MotorValue, float loopTime);
	float setControlSpeed(float MotorValue);
	bool controlOn;
	RobotDrive myRobot; // robot drive system
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	float MinPower;
	float RightMotorSpeed;
	float LeftMotorSpeed;
	float ArcadeMotorSpeed;
	float ArcadeRotateSpeed;
	Timer accelTimer;
};
#endif
