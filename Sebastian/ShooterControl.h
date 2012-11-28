#ifndef SHOOTERCONTROL_H
#define SHOOTERCONTROL_H

#include "WPILib.h"
#include "ShooterMotors.h"
#include "..\SampleLibrary\Controller.h"

class ShooterControl {
public:
	ShooterControl();
	void Run();
	void InitializeAutonomous();
	void InitializeOperator();
	void InitializeHybrid();
	void RunAuto();
	void RunEndGame();
	bool getLED1();
	bool getLED2();

private:
	ShooterMotors *_dsm;
	Controller *xbox;
	DriverStationLCD *dsLCDShoot;
	int currentShooterState;
	int shooterSpeedState;
	int autoCount;
	float shooterSpeed;
	void ShooterSpeedPrinter();
	void ShooterStateControl();
	void StateChecker();
	//void StoppedStateControl();
	void FeedMeStateControl();
	void FullStateControl();
	void ShootBotStateControl();
	void ShootTopStateControl();
	void PukeStateControl();
	void AutoPukeStateControl();
	void EndGameShoot();
	float GetShooterSpeed();
	float shootSpeedDiff;
};

#endif 
