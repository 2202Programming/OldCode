/* This is the Header File for Xbox Controller*/
#ifndef SHOOTERCONTROL_H
#define SHOOTERCONTROL_H

#include "WPILib.h"
#include <cmath>
#include "XboxController.h"

class ShooterControl {
public:
	~ShooterControl() {
	}
	static ShooterControl *getInstance();
	void initialize();
	void initializeAutonomous();
	void run();
	bool isRunning();
	void runAutonomous();
	void ShooterAngle(float angleDirection);
	void SetShooterMotors(float speed);
	float getAngle();
	
private:
	ShooterControl();

	#define SHOOTERMOTORPORT1 5
	#define SHOOTERMOTORPORT2 6
	#define SHOOTERANGLEMOTORPORT 7
	#define SHOOTERSPEEDSTEP .25
	#define UPPERLIMITPORT 7
	#define LOWERLIMITPORT 6
	#define ANGLEMOTORLIFTSPEED 0.3
	#define SHOOTERSPEEDINCREMENTRESETPT 0.0
	#define AUTOSPEED .2

	float Shooter1Speed;
	float Shooter2Speed;
	float shooterStartSpeed;
	float Angle;
	
	Jaguar* shooterMotor1;
	Jaguar* shooterMotor2;

	Victor* AngleMotor;
	Relay* AngleMotorRelay;
	
	XboxController *xbox;
	DriverStationLCD *dsLCD;

	DigitalInput *lowerLimit;
	DigitalInput *upperLimit;
	
	Accelerometer *accelerometer;

	void ShooterCycleSpeed();
	bool maxAngleReached();
};

#endif 
