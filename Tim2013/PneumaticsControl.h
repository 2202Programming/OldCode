#ifndef PNEUMATICSCONTROL_H
#define PNEUMATICSCONTROL_H

#include "WPILib.h"
#include "XboxController.h"

class PneumaticsControl {
public:
	~PneumaticsControl() {
	}
	PneumaticsControl();
	void initialize();
	void initializeAutonomous();
	void run();

private:
	
	Timer solenoidTimer;
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	Compressor *compressor;
	Solenoid *triggerSolenoid;
	Solenoid *retractSolenoid;
	void fire();
	bool firing;
 

};
#endif 
