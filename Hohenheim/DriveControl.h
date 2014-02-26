#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H

#include "WPILib.h"
#include <cmath>
#include "XboxController.h"
#include "PneumaticsControl.h"
#include "PIDControlSubClass.h"

class DriveControl {
public:
	DriveControl();
	void initialize();
	void initializeAuto();
	bool autoDrive(double autoDriveDistance);
	void run();
	bool autoPIDDrive();
	char*DriveControl::GetAutoStateString();
private:
	PneumaticsControl *pneumaticsControl;
	RobotDrive *myRobot;
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	Encoder *leftEncoder;
	Encoder *rightEncoder;
	/*
	 enum ShiftStates {
	 Init, Low, High, DelayToLow, DelayToHigh
	 };
	 ShiftStates shiftState;
	 int delayCount;
	 Timer shiftDelay;
	 */
	void runArcadeDrive();
	void manualShift();
	Talon * motorFrontLeft;
	Talon * motorBackLeft;
	Talon * motorFrontRight;
	Talon * motorBackRight;
	PIDControlSubClass* pIDControlOutputLeft;
	PIDController* controllerLeft;
	PIDControlSubClass* pIDControlOutputRight;
	PIDController* controllerRight;
	Timer autoTimer;
	double previousAutoTime;
	enum AutoState {
			AutoDrive, AutoStopped
	};
	AutoState currentAutoState;
	double autoDriveRampProfile(double timeChange);
};
#endif
