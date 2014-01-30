#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H

#include "WPILib.h"
#include <cmath>
#include "XboxController.h"
#include "PneumaticsControl.h"

class DriveControl {
public:
	DriveControl();
	void initialize();
	void initializeAutonomous();
	void runArcadeAutoShift();	
	void runShift();
	bool runAuto();


private:
	PneumaticsControl *pneumaticsControl;
	//float scaledOffset(float originalValue, float minValue);
	//float scaledOffset(float originalValue, float minValue, float maxValue);
	//float scaleValue(float originalValue, float offset);
	float accelerateMotor(float stickValue, float MotorValue, float loopTime);
	float accelerateTurnMotor(float stickValue, float MotorValue, float loopTime);
	float setControlSpeed(float MotorValue);
	bool precisionDrive;
	bool shiftDrive;
	Timer waitTime; 
	RobotDrive myRobot; // robot drive system	
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	Encoder *leftEncoder;
	Encoder *rightEncoder;
	float MinPower;
	float maxValue;
	float RightMotorSpeed;
	float LeftMotorSpeed;
	float ArcadeMotorSpeed;
	float ArcadeRotateSpeed;
	float counter;
	DigitalInput *lowerLimit;
	DigitalInput *upperLimit;
	Timer accelTimer;
	
	//Code for Manaul Shooting Set Speed
	Jaguar* UpperShooter;
	Jaguar* LowerShooter;
	 
	 
	
};
#endif
