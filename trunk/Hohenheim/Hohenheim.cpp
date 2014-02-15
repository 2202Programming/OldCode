/* Timothy robot,

 * Soaring disks throw to the goals.
 * Also, climbs. Maybe.
 */
#include "WPILib.h"
#include "Math.h"
#include "XboxController.h"
#include "DriveControl.h"
#include "PneumaticsControl.h"
#include "ShooterControl.h"

#define AUTODRIVETIME .5

class Hohenheim: public SimpleRobot {

	DriverStation *driverStation;
	DriverStationLCD *dsLCD;
	DriveControl driveControl;
	PneumaticsControl *pneumaticsControl;
	ShooterControl *shooterControl;

public:
	Hohenheim(void) {
		driverStation = DriverStation::GetInstance();
		dsLCD = DriverStationLCD::GetInstance();
		pneumaticsControl = PneumaticsControl::getInstance();
		shooterControl = ShooterControl::getInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Hohenheim 2014 V 2.3");
		dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(false);

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
		SimpleAutonomous();

	}

	void SimpleAutonomous() {

		pneumaticsControl->initialize();
		shooterControl->initializeAuto();
		Timer driveTime;
		float waitTime = 1.0; // Time we drive the robot
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Autonomous control");
		dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(true);

		bool driveNow = false;
		driveTime.Start();

		while (IsAutonomous() && IsEnabled()) {
			GetWatchdog().Feed();
			pneumaticsControl->ballGrabberExtend();
			if (pneumaticsControl->ballGrabberIsExtended()){
				shooterControl->autoShoot();
			}
			if (shooterControl->doneAutoFire()) {
				if (driveTime.Get() < waitTime) {
					driveNow = true;
				} else if (driveTime.Get() > waitTime) {
					driveNow = false;
					driveTime.Stop();
				}

			}
			driveControl.autoDrive(driveNow);

		}
	}

	void OperatorControl(void) {
		GetWatchdog().SetEnabled(true);
		dsLCD->Clear();
		//dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Operator control");
		dsLCD->UpdateLCD();
		driveControl.initialize();
		pneumaticsControl->initialize();
		shooterControl->initialize();
		while (IsOperatorControl() && IsEnabled()) {
			GetWatchdog().Feed();
			driveControl.run();
			pneumaticsControl->run();
			shooterControl->run();
			dsLCD->UpdateLCD();
			Wait(0.005); // wait for a motor update time
		}
	}

};

START_ROBOT_CLASS(Hohenheim)
;

