#ifndef SHOOTERCONTROL_H
#define SHOOTERCONTROL_H

#include "WPILib.h"
#include "XboxController.h"
#include "PneumaticsControl.h"
#include "PIDControlSubClass.h"
#include "IControl.h"

class ShooterControl: public IControl {
public:
	~ShooterControl() {
	}
	static ShooterControl *getInstance();
	ShooterControl();
	void initialize();
	void initializeAuto();
	void autoShoot(); //for autonomous mode
	void ballGrabber();
	void PIDShooter();
	void ManualShoot();
	void run();
	void autoLoad(bool on);
	void toggleColor();
	void feed(bool toggleMotor);
	void runAuto(){}

	char*GetStateString();
	char*GetAutoStateString();
	bool doneAutoFire();

private:
	double downRampProfile(double timeChange);
	double shootRampProfile(double timeChange);
	double loadRampProfile(double timeChange);
	bool canIFire();
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
	float maxAutoShootTime;
	int limitCount;
	int maxEncoderValue;
	enum fireStates {
		Arming, ReadyToFire, Firing, Init, Fired, Retracting, Home, Passing, TrussShot ,TrussSetup , StageOneFire, StageTwoFire
	};
	enum autoFireStates {
		AutoInit, GoHome, AutoWait, AutoFire , AutoRetract
	};
	autoFireStates autoFireState;
	fireStates fireState;
	bool loadingBall;
	DigitalInput *lowerLimit;
	DigitalInput *upperLimit;
	DigitalInput *five;
	Timer autoShootTimer;
	Timer shooterTimer;
	double previousTime;
	bool autoShot;
	bool doneAutoFired;
	int lightCounter;
	Relay *LED1;
	Relay *LED2;
	Relay *LED3;
	double cummulativeTime;
//float TRUSSSETUP 15
//float TRUSSPIDSETUP -0.3
//float TRUSSPIDFIRE  1.00
//float TRUSS 110 
	int twoStageSetupPosition;
	float twoStagePidSetup;
	int twoStageEndPosition;
	float twoStagePidFire;
};
#endif
