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
	
private:
	PneumaticsControl();
	Timer autoTimer;
	Timer solenoidTimer;
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	Compressor *compressor;
	Solenoid *rightTrigger;
	//Solenoid *leftTrigger;
	Solenoid *rightRetract;
	//Solenoid *leftRetract;
	bool shiftState;
	bool highGear;

};
#endif 
