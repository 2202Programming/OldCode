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
	bool isLowestAngle();

private:
	ShooterControl();

#define SHOOTERMOTORPORT1 5
#define SHOOTERMOTORPORT2 6
#define SHOOTERANGLEMOTORPORT 7
#define SHOOTERSPEEDSTEP .2
#define UPPERLIMITPORT 7
#define LOWERLIMITPORT 6
#define ANGLEMOTORLIFTSPEED 0.3
#define SHOOTERSPEEDINCREMENTRESETPT 0.0
#define AUTOSPEED .2
#define SHOOTERSPEEDSTEPDIFF .1
#define DEFAULTSHOOTERSPEED 1.0
#define SHOOTERLOADINGSPEED 0.0
#define SHOOTERFIRINGSPEED .99
#define SHOOTERLOADINGDIRECTION -1.0 // these are for raising and lowering
#define SHOOTERFIRINGDIRECTION .99

	float Shooter1Speed;
	float Shooter2Speed;
	float shooterStartSpeed;
	float Angle;
	bool isLoadingPosition;
	bool isFiringPosition;

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
	void ShooterSeperateCycleSpeed();
	bool maxAngleReached();
	void Stop();
	void APushStop();
	void ShooterCycleBehindSpeed();
	void goTo();
};

#endif 
