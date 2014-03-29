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
	void BeastMode();
	bool autoPIDDrive();
	char*DriveControl::GetAutoStateString();
	bool autoPIDDrive2();
	void stickLimiter(float stick_X, float stick_Y);
	
private:
	float totalPositionChange;
	PneumaticsControl *pneumaticsControl;
	RobotDrive *myRobot;
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	Encoder *leftEncoder;
	Encoder *rightEncoder;
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
	float autoDriveRampProfile(float timeChange);
	float SpeedControl; 
	bool beastMode;
	float stick_Prev_X;
	float stick_Prev_Y;
	float stick_X_Cmd;
	float stick_Y_Cmd;
};
#endif
