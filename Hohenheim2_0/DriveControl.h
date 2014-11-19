#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H

#include "WPILib.h"
#include <cmath>
#include "XboxController.h"
#include "PneumaticsControl.h"
#include "PIDControlSubClass.h"
#include "IControl.h"

class DriveControl: public IControl {
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
	void runAuto(){}

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
	Timer shiftTimer;
	double previousAutoTime;
	enum AutoState {
		AutoDrive, AutoStopped
	};
	enum ShiftStates {
		Idle, ShiftWait, ShiftComplete
	};
	ShiftStates currentShiftState;
	AutoState currentAutoState;
	float autoDriveRampProfile(float timeChange);
	float SpeedControl;
	bool beastMode;
	bool ShiftingUp;
	float stick_Prev_X;
	float stick_Prev_Y;
	float stick_X_Cmd;
	float stick_Y_Cmd;
};
#endif
