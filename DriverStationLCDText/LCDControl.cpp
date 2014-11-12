#include "LCDControl.h"


	LCDControl::LCDControl() {
		dsLCD = DriverStationLCD::GetInstance();
	}
	void LCDControl::initialize(){
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "In initialize");
	}
	void LCDControl::initializeAuto(){
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "In initializeAuto");
	}
	void LCDControl::run(){
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "In run");
	}
	void LCDControl::runAuto(){
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "In runAuto");
	}
