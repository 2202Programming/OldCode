#ifndef SHOOTERCONTROL_H
#define SHOOTERCONTROL_H

#include "WPILib.h"
#include "XboxController.h"
#include "PneumaticsControl.h"
#include "PIDControlSubClass.h"


class ShooterControl {
public:
	~ShooterControl() {
	}
	static ShooterControl *getInstance();
	ShooterControl();
	void initialize();
	void ballGrabber();
	void PIDShooter();
	void ManualShoot();
	void run();
	double downRampProfile (double Time);
	char*GetStateString();

private:
	PneumaticsControl *pneumaticsControl;
	float accelerateMotor(float stickValue, float MotorValue, float loopTime);
	float accelerateTurnMotor(float stickValue, float MotorValue,
			float loopTime);
	float setControlSpeed(float MotorValue);
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	Talon* upperShooter;
	Talon* lowerShooter;
	Talon* upperShooter2;
	Talon* lowerShooter2;
	Talon* BallGrabberMotor5;
	Talon* BallGrabberMotor6;
	Encoder *shooterEncoder;
	//Encoder* input;
	PIDControlSubClass* pIDControlOutput;
	PIDController* controller;
	bool isLeft;
	float Kp;
	float Ki;
	float Kd;
	float MinPower;
	float maxValue;
	float counter;
	int limitCount;
	enum fireStates {
		Arming, ReadyToFire, Firing, Init, Fired,
	};
	bool canIFire();
	fireStates fireState;
	bool loadingBall;
	DigitalInput *lowerLimit;
	DigitalInput *upperLimit;
	DigitalInput *five;
	Timer accelTimer;
	Timer shooterTimer; 
	double previousTime; 
};
#endif
