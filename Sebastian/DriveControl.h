#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H
#define PID_DT 4 
#define PID_KP 1.0
#define PID_KI .8
#define PID_KD .8
#include "WPILib.h"
#include "SonarSensor.h"
#include <cmath>
#include "..\SampleLibrary\Controller.h"
class DriveControl {
public:
	DriveControl();
	bool tipBridgeAuto();
	
	void act();
	void initialize();
	void initializeAutonomous();
	void initializeHybrid();

	bool RunAuto();
	void actEndGame();

private:
	float scaledOffset(float originalValue, float minValue);
	float scaledOffset(float originalValue, float minValue,float maxValue);
	float scaleValue(float originalValue, float offset);
	bool isDead(float value);
	float adjustDeadband(float value);
	void MoveForward(float & lefty, float & righty);
	RobotDrive myRobot; // robot drive system
	Controller *xbox;
	float DeadbandWidth;
	float MinPower;
	SonarSensor sonarCenter;
	AnalogChannel signalControlVoltage;
	DriverStationLCD *dsLCD;
	float newX;
	float oldX;
	int currentDirection;
	int nextState;
	int commandState;
	float referenceHigh;
	float referenceLow;
	int count;
	int autoCount;
	Accelerometer accelFront;
	Accelerometer accelBack;
	float endGameMaxValue;

};
#endif
