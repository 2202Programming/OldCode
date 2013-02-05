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
	
	
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	Compressor *compressor;
	Solenoid *triggerSolenoid;
	Solenoid *retractSolenoid;
	void fire();

 

};
#endif 
