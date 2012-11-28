#include "PoleVaultControl.h"

#define DEPLOY_STATE 3278
#define RETRACT_STATE 24156
#define DEFAULT_STATE 437289 

PoleVaultControl::PoleVaultControl() :
	_retractPoleVault(2, 2), _deployPoleVault(2, 1) {
	_compressor = new Compressor(1, 1);
	_retractPoleVault.Set(false);
	_deployPoleVault.Set(false);
	xbox = XboxController::getInstance();
	dsLCD = DriverStationLCD::GetInstance();
	currentState = RETRACT_STATE;
	compressorStarted = false;
}

void PoleVaultControl::stateManager() {
	if (xbox->isBackPressed()) {
		switch (currentState) {
		default:
//		case DEFAULT_STATE:
//			currentState = DEPLOY_STATE;
//			break;
		case RETRACT_STATE:
			currentState = DEPLOY_STATE;
			break;
		case DEPLOY_STATE:
			currentState = RETRACT_STATE;
			break;
		}
	}
}

void PoleVaultControl::initialize() {
	if (!compressorStarted) {
		_compressor->Start();
		compressorStarted = true;
	}
}

void PoleVaultControl::defaultControl() {
	_retractPoleVault.Set(false);
	_deployPoleVault.Set(false);
}

void PoleVaultControl::retractControl() {
	_retractPoleVault.Set(true);
	_deployPoleVault.Set(false);
}

void PoleVaultControl::deployControl() {
	_retractPoleVault.Set(false);
	_deployPoleVault.Set(true);
}

void PoleVaultControl::act() {
	char *stateString;
	stateManager();
	switch (currentState) {
	default:
//	case DEFAULT_STATE:
//		stateString = "Default";
//		defaultControl();
//		break;
	case RETRACT_STATE:
		stateString = "Retract";
		retractControl();
		break;
	case DEPLOY_STATE:
		stateString = "Deploy";
		deployControl();
		break;
	}
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "%8s", stateString);
}

void PoleVaultControl::actEndGame(){
	act();
}

void PoleVaultControl::deploy(){
	deployControl();
}
