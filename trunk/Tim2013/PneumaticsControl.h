#ifndef PNEUMATICSCONTROL_H
#define PNEUMATICSCONTROL_H

#include "WPILib.h"
#include "XboxController.h"

#define SHOOTERBOOTTIME 2.0
#define SHOOTERRESTPERIOD 3.0

class PneumaticsControl {
public:
	~PneumaticsControl() {
	}
	PneumaticsControl();
	void initialize();
	//void initializeAutonomous();
	void run();
//	void runAutonomous();
	bool CompressorFull();
	void fire();
	void springFire();
	void autoFire();
private:
	
	Timer solenoidTimer;
	//Timer autonomousTimer;
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	Compressor *compressor;
	Solenoid *triggerSolenoid;
	Solenoid *retractSolenoid;
	bool firing;
	ShooterControl *shooterControl;

};
#endif 
