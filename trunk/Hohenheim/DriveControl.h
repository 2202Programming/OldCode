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
	bool autoDrive(double autoDriveDistance);
	void run();

private:
	PneumaticsControl *pneumaticsControl;
	RobotDrive myRobot; 
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

};
#endif
