/* Timothy robot,
 * Soaring disks throw to the goals.
 * Also, climbs. Maybe.
 */
#include "WPILib.h"
#include "Math.h"
#include "XboxController.h"
#include "DriveControl.h"
#include "ShooterControl.h"
#include "PneumaticsControl.h"
#include "LiftControl.h"

class Tim2013: public SimpleRobot {

	DriverStationLCD *dsLCD;
	DriveControl driveControl;
	ShooterControl *shooterControl;
	PneumaticsControl pneumaticsControl;
	LiftControl *liftControl;
	Relay *ledRelay;
public:
	Tim2013(void) {
		shooterControl = ShooterControl::getInstance();
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Tim 2013 V 0.4.6");
		dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(false);
		liftControl = LiftControl::getInstance();
		
		ledRelay = new Relay(8, Relay::kForwardOnly);
		ledRelay->Set(Relay::kOn);
	}

	void Autonomous(void) {
		SimpleAutonomous();
	}

	void OperatorControl(void) {
		GetWatchdog().SetEnabled(true);

		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Operator control");
		dsLCD->UpdateLCD();

		driveControl.initialize();
		pneumaticsControl.initialize();
		liftControl->initialize();
		while (IsOperatorControl() && IsEnabled()) {
			GetWatchdog().Feed();

			driveControl.runArcade();
			shooterControl->run();
			pneumaticsControl.run();
			liftControl->run();
			Wait(0.005); // wait for a motor update time
		}
	}

	void SimpleAutonomous() {
		//		Start the Compressor
		pneumaticsControl.initialize();
		Timer fireRate;
		float waitTime = 3.0; // time before ready to shoot
		fireRate.Start();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Autonomous control");
					dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(true);
		
		while (IsAutonomous()&&IsEnabled()) {
			GetWatchdog().Feed();
			//	Start the Angle to go until it is at the top
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "while loop");
					dsLCD->UpdateLCD();
			//shooterControl->ShooterAngle(1);
			//	run motors from ShooterControl
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "angle set");
					dsLCD->UpdateLCD();
			shooterControl->SetShooterMotors(.4);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "set motors");
					dsLCD->UpdateLCD();
			//	check if angle has reached desired
			if (shooterControl->getAngle() == 1.0) {dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "if angle");
			dsLCD->UpdateLCD();
				if (fireRate.Get() > waitTime) {
					waitTime = 1.5; // change wait time to fire rate
					pneumaticsControl.autoFire();
					fireRate.Reset();
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "fire");
							dsLCD->UpdateLCD();
				}
			}
		}
	}

};

START_ROBOT_CLASS(Tim2013)
;

