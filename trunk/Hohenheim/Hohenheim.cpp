/* Timothy robot,

 * Soaring disks throw to the goals.
 * Also, climbs. Maybe.
 */
#include "WPILib.h"
#include "Math.h"
#include "XboxController.h"
#include "DriveControl.h"
#include "PneumaticsControl.h"

#define AUTODRIVETIME .5

class Hohenheim: public SimpleRobot {

	DriverStation *driverStation;
	DriverStationLCD *dsLCD;
	DriveControl driveControl;
	PneumaticsControl *pneumaticsControl;
	
public:
	Hohenheim(void) {
		driverStation = DriverStation::GetInstance();
		dsLCD = DriverStationLCD::GetInstance();
		pneumaticsControl = PneumaticsControl::getInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Hohenheim 2014 V 2.0 shift test");
		dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(false);
;
	}

	void DashBoardInput() {
		int i = 0;
		GetWatchdog().SetEnabled(true);
		while (IsAutonomous() && IsEnabled()) {
			i++;
			GetWatchdog().Feed();
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "%f, %i",
					driverStation->GetAnalogIn(1), i);
			dsLCD->UpdateLCD();

		}
	}

	void Autonomous(void) {
	

	}

	void OperatorControl(void) {
		GetWatchdog().SetEnabled(true);
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Operator control");
		dsLCD->UpdateLCD();
		driveControl.initialize();
		pneumaticsControl->initialize();
		while (IsOperatorControl() && IsEnabled()) {
			GetWatchdog().Feed();
			driveControl.runArcadeNoAcceleration();
			pneumaticsControl->run();
			Wait(0.005); // wait for a motor update time
		}
	}



};

START_ROBOT_CLASS(Hohenheim)
;

