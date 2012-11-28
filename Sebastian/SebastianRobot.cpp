#include "WPILib.h"
#include "ShooterControl.h"
#include "DriveControl.h"
#include "PoleVaultControl.h"
#include "..\SampleLibrary\Controller.h"
#include "..\SampleLibrary\XboxController.h"

#define WAIT_TIME 0.005
#define FLASHTIME 100;

class SebastianRobot: public SimpleRobot {
	ShooterControl shooterControl;
	DriveControl _driveControl;
	PoleVaultControl _poleVaultControl;
	DriverStationLCD *dsLCD;
	Relay *led0;
	Relay *led1;
	int flashCount;
	bool isFlashing;

public:
	SebastianRobot(void) {
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Sebastian V24.2");
		dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(false);
		led0 = new Relay(2);
		led1 = new Relay(3);
		flashCount = 0;
		led0->Set(Relay::kOff);
		led1->Set(Relay::kOff);
		isFlashing=false;
	}

	void Autonomous(void) {
		GetWatchdog().SetEnabled(true);
		bool isHybrid = false;
		Kinect* kinect = Kinect::GetInstance();
		isHybrid = (kinect->GetNumberOfPlayers() > 0);
		if (!isHybrid) {
			_driveControl.initializeAutonomous();
			shooterControl.InitializeAutonomous();
			_poleVaultControl.initialize();
		} else {
			_driveControl.initializeHybrid();
			shooterControl.InitializeHybrid();
			_poleVaultControl.initialize();
		}
		while (IsEnabled() && IsAutonomous()) {
			GetWatchdog().Feed();
			dsLCD->Clear();
			if (!isHybrid) { //Run Autonomous
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1,
						"Autonomous Mode");
				//if (_driveControl.RunAuto()) {
					shooterControl.RunAuto();
				//}
//				if(_driveControl.RunAuto()){
//					_poleVaultControl.deploy();
//				}
			} else { //Run Hybrid
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Hybrid Mode");
				shooterControl.Run();
				_driveControl.act();
				_poleVaultControl.act();
			}
			dsLCD->UpdateLCD();
			Wait(WAIT_TIME);
		}
		GetWatchdog().SetEnabled(false);
	}

	void OperatorControl(void) {
		XboxController *xbox = XboxController::getInstance();

		bool isEndGame = false;
		GetWatchdog().SetEnabled(true);
		_driveControl.initialize();
		//_poleVaultControl.initialize();
		shooterControl.InitializeOperator();
		while (IsEnabled() && IsOperatorControl()) {
			GetWatchdog().Feed();
			dsLCD->Clear();
			if (xbox->isEndGame()) {
				isEndGame = !isEndGame;
			}
			if (!isEndGame) {
				shooterControl.Run();
				//_poleVaultControl.act();
				_driveControl.act();
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Normal");
				led0->Set((shooterControl.getLED1())?Relay::kOn: Relay::kOff);
				led1->Set((shooterControl.getLED2())?Relay::kOn: Relay::kOff);
			}

			else {
				shooterControl.RunEndGame();
				//_poleVaultControl.actEndGame();
				_driveControl.actEndGame();
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "End Game");
				
				flashCount--;
				
				if (flashCount<=0){
					isFlashing = !isFlashing;
					flashCount=FLASHTIME;
				
				}
				
				
				led0->Set((isFlashing)?Relay::kOn: Relay::kOff);
				led1->Set((isFlashing)?Relay::kOn: Relay::kOff);
			}
//			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "Flash: %i", flashCount);
			dsLCD->UpdateLCD();
			Wait(WAIT_TIME); // wait for a motor update time

		}

		GetWatchdog().SetEnabled(false);
	}
};

START_ROBOT_CLASS(SebastianRobot)
;
