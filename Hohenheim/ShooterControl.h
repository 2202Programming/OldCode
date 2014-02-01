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
	void shoot();
	void PIDShooter();
	void twoStageShoot();
	void run();

private:
	PneumaticsControl *pneumaticsControl;
	float accelerateMotor(float stickValue, float MotorValue, float loopTime);
	float accelerateTurnMotor(float stickValue, float MotorValue,
			float loopTime);
	float setControlSpeed(float MotorValue);
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	Talon* UpperShooter;
	Talon* LowerShooter;
	Victor* BallGrabberMotor5;
	Victor* BallGrabberMotor6;
	Encoder *shooterEncoder;
	//Encoder* input;
	PIDControlSubClass* output;
	PIDController* controller;
	bool isLeft;
	float Kp;
	float Ki;
	float Kd;
	float MinPower;
	float maxValue;
	float counter;
	enum fireStates {
		Arming, ReadyToFire, Firing,
	};
	enum twoStageFire{
		Rest, Fired,
	};
	twoStageFire twoStageFire;
	fireStates fireState;
	bool loadingBall;
	DigitalInput *lowerLimit;
	DigitalInput *upperLimit;
	Timer accelTimer;
};
#endif
