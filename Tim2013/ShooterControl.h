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
	ShooterControl();
	void initialize();
	void initializeAutonomous();
	void run();

private:
	
	Jaguar* shooterMotor1;
	Jaguar* shooterMotor2;
	Jaguar* AngleMotor;
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	DigitalInput *lowerLimit;
	DigitalInput *upperLimit;
    void ShooterCycleSpeed();
    void ShooterAngle();

    //the following are the varriables use in experiment method
    float lShooterSpeed;
    float rShooterSpeed;
   


};

#endif 
