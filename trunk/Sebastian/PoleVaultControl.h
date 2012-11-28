#ifndef POLEVAULTCONTROL_H
#define POLEVAULTCONTROL_H

#include "WPILib.h"
#include "..\SampleLibrary\XboxController.h"
#include "..\SampleLibrary\Controller.h"

class PoleVaultControl {
public:
	PoleVaultControl();
	void act();
	void initialize();
	void actEndGame();
	void deploy();
private:
	void stateManager();
	void retractControl();
	void deployControl();
	void defaultControl();
	Solenoid _retractPoleVault;
	Solenoid _deployPoleVault;
	Compressor *_compressor;
	Controller *xbox;
	DriverStationLCD *dsLCD;
	int currentState;
	bool compressorStarted;
};

#endif
