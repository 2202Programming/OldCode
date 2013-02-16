/* This is the Header File for Xbox Controller*/
#ifndef LIFTCONTROL_H
#define LIFTCONTROL_H



#include "WPILib.h"
#include <cmath>
#include "XboxController.h"



#define TOPCHANNEL 3
#define BOTCHANNEL 4
#define SOLENOIDMODULE 1


class LiftControl {
public:
	~LiftControl() {
	}
	static LiftControl *getInstance();
	void initialize();
	void initializeAutonomous();
	void run();
	void runAutonomous();
private:
	LiftControl();


	XboxController *xbox;
	DriverStationLCD *dsLCD;

	Solenoid *solenoidTop;
	Solenoid *solenoidBot;

	bool isGoingUp;
};

#endif 
