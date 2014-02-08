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

	void run();

private:
	PneumaticsControl *pneumaticsControl;
	//float scaledOffset(float originalValue, float minValue);
	//float scaledOffset(float originalValue, float minValue, float maxValue);
	//float scaleValue(float originalValue, float offset);
	float accelerateMotor(float stickValue, float MotorValue, float loopTime);
	float accelerateTurnMotor(float stickValue, float MotorValue,
			float loopTime);
	float setControlSpeed(float MotorValue);
	bool precisionDrive;
	bool shiftDrive;
	Timer shiftDelay;
	RobotDrive myRobot; // robot drive system	
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	Encoder *leftEncoder;
	Encoder *rightEncoder;
	Talon* UpperShooter;
	Talon* LowerShooter;
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
	void runArcadeDrive();
	//bool runAuto();
	void Shifter();
	enum ShiftStates {
		Init, Low, High, DelayToLow, DelayToHigh
	};
	ShiftStates shiftState;
	void setLowGear();
	void setHighGear();
	int delayCount;
	
};
#endif
