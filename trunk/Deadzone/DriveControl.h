/* This is the Header File for Xbox Controller*/
#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H
#include "WPILib.h"
#include "Nivision.h"
#include "XboxController.h"
#include "SonarSensor.h"
#include <cmath>
class DriveControl {
public:
	DriveControl();
	void act();
	void initialize();
private:
	float scaledOffset(float originalValue, float offset);
	float scaleValue(float originalValue, float offset);
	bool isDead(float value);
	float adjustDeadband(float value);
	RobotDrive myRobot; // robot drive system
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	float DeadbandWidth;
	float MinPower;
	SonarSensor sonarCenter;
	AnalogChannel signalControlVoltage;
	float newX;
	float oldX;
	int currentDirection;
	int nextState;
	int commandState;
	float referenceHigh;
	float referenceLow;
	int count;
};
#endif
