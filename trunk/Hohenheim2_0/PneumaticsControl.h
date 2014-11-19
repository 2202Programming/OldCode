#ifndef PNEUMATICSCONTROL_H
#define PNEUMATICSCONTROL_H

#include "IControl.h"
#include "WPILib.h"
#include "XboxController.h"

#define SHOOTERBOOTTIME 2.0
#define SHOOTERRESTPERIOD 3.0

class PneumaticsControl : public IControl{

public:
	//~PneumaticsControl() {
	
	static PneumaticsControl *getInstance();
	//from IControl
	void initialize();
	void initializeAuto(){}
	void run(){}
	void runAuto(){}
	//void run();
	//bool CompressorFull();
	//void shift();
	void shiftUp();
	void shiftDown();
	bool isHighGear();
	bool ballGrabberIsExtended();
	void disable();
	void ballGrabberExtend();
	void ballGrabberRetract();
	void compressorEnable();
	void compressorDisable();
	int ReadSwitch();
	void ballGrabberToggle() {
		if (isBallGrabberExtended) {
			ballGrabberRetract();
		} else {
			ballGrabberExtend();
		}
	}

private:
	PneumaticsControl();
	Timer autoTimer;
	Timer solenoidTimer;
	XboxController *xbox;
	DriverStationLCD *dsLCD;
	Compressor *compressor;
	/*
	 //For shifting
	 Solenoid *rightTrigger;
	 Solenoid *leftTrigger;
	 Solenoid *rightRetract;
	 Solenoid *leftRetract;
	 
	 //For ball pick up
	 Solenoid *extendBallGrabberR;
	 Solenoid *extendBallGrabberL;
	 Solenoid *retractBallGrabberR;
	 Solenoid *retractBallGrabberL;
	 */
	bool shiftState;
	bool highGear;
	bool isBallGrabberExtended;

	DoubleSolenoid *shiftControlL;
	DoubleSolenoid *shiftControlR;
	DoubleSolenoid *ballGrabberControlL;
	DoubleSolenoid *ballGrabberControlR;
	DigitalInput *ballGrabberExtendLimit;
	


};
#endif 
