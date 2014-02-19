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
#define AUTODRIVEDISTANCE 60.0

class Hohenheim: public SimpleRobot {

	DriverStation *driverStation;
	DriverStationLCD *dsLCD;
	DriveControl driveControl;
	PneumaticsControl *pneumaticsControl;
	ShooterControl *shooterControl;
	bool autoShot;

public:
	Hohenheim(void) {
		driverStation = DriverStation::GetInstance();
		dsLCD = DriverStationLCD::GetInstance();
		pneumaticsControl = PneumaticsControl::getInstance();
		shooterControl = ShooterControl::getInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Hohenheim 2014 V 3.0");
		dsLCD->UpdateLCD();
		autoShot = false; //true if autoShoot called in autonmous
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
		//ShootThenDriveAuto(); //Auto Mode to shoot then move foward
		//DriveAuto(); 		  		//Auto Mode to move foward 
		//AimShootThenDriveAuto();  //Auto Mode to aim shoot then move foward
		DriveThenShootAuto();

	}
	
	/*
	void DriveAuto() {
		pneumaticsControl->initialize();
		Timer driveTime;
		float waitTime = 2.0; // Time we drive the robot
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Autonomous control");
		dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(true);
		bool driveNow = false;
		while (IsAutonomous() && IsEnabled()) {
			GetWatchdog().Feed();
			driveTime.Start();
			driveControl.autoDrive(driveNow);
			if (driveTime.Get() < waitTime) {
				driveNow = true;
			} else if (driveTime.Get() > waitTime) {
				driveNow = false;
				driveTime.Stop();
			}
		
		}
	}*/

	void DriveThenShootAuto() {
		pneumaticsControl->initialize();
		shooterControl->initializeAuto();
		driveControl.initialize();
		while (IsAutonomous() && IsEnabled()) {
			GetWatchdog().Feed();
			pneumaticsControl->ballGrabberExtend();
			bool atDestination = driveControl.autoDrive(AUTODRIVEDISTANCE); //autoDrive returns true when 
			if(atDestination){
				if(pneumaticsControl->ballGrabberIsExtended() && !autoShot){
					shooterControl->autoShoot();
					autoShot = true;
				}
			}
		}
	}
	
	/*
	void ShootThenDriveAuto() {
		pneumaticsControl->initialize();
		shooterControl->initializeAuto();
		Timer driveTime;
		float waitTime = 2.0; // Time we drive the robot
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Autonomous control");
		dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(true);
		bool driveNow = false;
		Timer shootWaitTime;
		bool shootIsOn = false;
		while (IsAutonomous() && IsEnabled()) {
			GetWatchdog().Feed();
			pneumaticsControl->ballGrabberExtend();
			if (shooterControl->doneAutoFire()) {
				driveTime.Start();
				if (driveTime.Get() < waitTime) {
					driveNow = true;
				} else if (driveTime.Get() > waitTime) {
					driveNow = false;
					driveTime.Stop();
				}
			} else {
				if (pneumaticsControl->ballGrabberIsExtended()) {
					if (!shootIsOn) {
						shootWaitTime.Start();
						shootIsOn = true;
					}
					if (shootWaitTime.Get() > waitTime) {
						shooterControl->autoShoot();
						shootWaitTime.Stop();
						shootWaitTime.Reset();
					}
				}
			}
			driveControl.autoDrive(driveNow);

		}
	}*/

	void OperatorControl(void) {
		GetWatchdog().SetEnabled(true);
		dsLCD->Clear();
		dsLCD->UpdateLCD();
		driveControl.initialize();
		pneumaticsControl->initialize();
		shooterControl->initialize();
		while (IsOperatorControl() && IsEnabled()) {
			GetWatchdog().Feed();
			driveControl.run();
			shooterControl->run();
			dsLCD->UpdateLCD();
			Wait(0.005); // wait for a motor update time
		}
		pneumaticsControl->disable();
	}

};

START_ROBOT_CLASS(Hohenheim)
;

