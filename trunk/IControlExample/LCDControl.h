#ifndef LCDCONTROL_H
#define LCDCONTROL_H

#include "WPILib.h"
#include "IControl.h"


class LCDControl : public IControl {
public:
	LCDControl();
	void initialize();
	void initializeAuto();
	void run();
	void runAuto();
	
private:
	DriverStationLCD *dsLCD;
};
#endif
