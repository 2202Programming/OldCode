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
	static PneumaticsControl *getInstance();
	void initialize();
	void run();
	bool CompressorFull();
	void shift();
	void shiftUp();
	void shiftDown();
	bool isHighGear();
	bool isPistonOn();
	void piston();
	void pistonOn();
	void pistonOff();
	
private:
	PneumaticsControl();
	Timer autoTimer;
	Timer solenoidTimer;
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	Compressor *compressor;
	
	//For shifting
	Solenoid *rightTrigger;
	Solenoid *leftTrigger;
	Solenoid *rightRetract;
	Solenoid *leftRetract;
	
	//For ball pick up
	Solenoid *pistonR;
	Solenoid *pistonL;
	Solenoid *retractPistonR;
	Solenoid *retractPistonL;
	
	bool shiftState;
	bool highGear;
	bool pistonState;

};
#endif 
