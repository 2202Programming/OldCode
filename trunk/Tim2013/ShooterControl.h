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
	
	#define SHOOTERMOTORPORT1 5
	#define SHOOTERMOTORPORT2 6
	#define SHOOTERANGLEMOTORPORT 7
	#define SHOOTERSPEEDSTEP .1
	#define UPPERLIMITPORT 2
	#define LOWERLIMITPORT 1
	#define ANGLEMOTORLIFTSPEED 0.3
	#define SHOOTERSPEEDINCREMENTRESETPT 0.0

	float Shooter1Speed;
	float Shooter2Speed;
	Jaguar* shooterMotor1;
	Jaguar* shooterMotor2;
	
	Jaguar* AngleMotor;

	XboxController *xbox;
	DriverStationLCD *dsLCD;

	DigitalInput *lowerLimit;
	DigitalInput *upperLimit;
    
	void ShooterCycleSpeed();
    void ShooterAngle();

   


};

#endif 
