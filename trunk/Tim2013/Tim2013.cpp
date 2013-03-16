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
#include "StringPot.h"

#define AUTODRIVETIME .5
//#include "LiftControl.h"

class Tim2013: public SimpleRobot {

	DriverStation *driverStation;
	DriverStationLCD *dsLCD;
	DriveControl driveControl;
	ShooterControl *shooterControl;
	PneumaticsControl pneumaticsControl;
	//LiftControl *liftControl;
	Relay *ledRelay;
	StringPot *stringPot;
public:
	Tim2013(void) {
		driverStation = DriverStation::GetInstance();
		shooterControl = ShooterControl::getInstance();
		stringPot = StringPot::getInstance();
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Tim 2013 V 1.2");
		dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(false);
		//liftControl = LiftControl::getInstance();

		ledRelay = new Relay(8, Relay::kForwardOnly);
		ledRelay->Set(Relay::kOn);
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
		//DashBoardInput();
		AnalogAutonomous();
		//SimpleAutonomous();

	}

	void OperatorControl(void) {
		GetWatchdog().SetEnabled(true);
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Operator control");
		dsLCD->UpdateLCD();
		shooterControl->initialize();
		driveControl.initialize();
		pneumaticsControl.initialize();
		//liftControl->initialize();
		while (IsOperatorControl() && IsEnabled()) {
			GetWatchdog().Feed();
			driveControl.runArcadeNoAcceleration();
			shooterControl->run();
			pneumaticsControl.run();
			stringPot->run();
			//	liftControl->run();
			Wait(0.005); // wait for a motor update time
		}
	}

	void SimpleAutonomous() {
		//		Start the Compressor

		pneumaticsControl.initialize();
		Timer fireRate;
		float waitTime = 3.0; // time before ready to shoot

		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Autonomous control");
		dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(true);
/*		if (!driverStation->GetDigitalIn(2)) {
			Timer driveTime;
			driveTime.Start();
			while (IsAutonomous() && IsEnabled() && driveTime.Get()
					< AUTODRIVETIME) {
				GetWatchdog().Feed();
				driveControl.runAuto();
			}
			driveTime.Stop();
		}
*/
		bool fireNow = false;
		fireRate.Start();
		while (IsAutonomous() && IsEnabled()) {
			GetWatchdog().Feed();

			//	Start the Angle to go until it is at the top
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "while loop");
			dsLCD->UpdateLCD();

			shooterControl->ShooterAngle(1);

			//	run motors from ShooterControl
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "angle set");
			dsLCD->UpdateLCD();

			shooterControl->SetShooterMotors(1.0);

			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "set motors");
			dsLCD->UpdateLCD();
			//	check if angle has reached desired
			fireNow = false;
			if (shooterControl->getAngle() == 1.0) {
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "if angle");
				dsLCD->UpdateLCD();

				if (fireRate.Get() > waitTime) {
					waitTime = .5; // change wait time to fire rate
					fireNow = true;
					fireRate.Reset();

				}
			}
			pneumaticsControl.autoFire(fireNow);
		}
	}

	// autonomous firing at speed set by analog input 1
	void AnalogAutonomous() {
		//		Start the Compressor
		
		pneumaticsControl.initialize();
		Timer fireRate;
		float waitTime = 3.0; // time before ready to shoot

		float fireSpeed = driverStation->GetAnalogIn(1);
		if (fireSpeed > 1.0) {
			fireSpeed = 1.0;
		}
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Autonomous control");
		dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(true);
/*		if (!driverStation->GetDigitalIn(2)) {
			Timer driveTime;
			driveTime.Start();
			while (IsAutonomous() && IsEnabled() && driveTime.Get()
					< AUTODRIVETIME) {
				GetWatchdog().Feed();
				driveControl.runAuto();
			}
			driveTime.Stop();
		}*/
		bool fireNow = false;
		fireRate.Start();
		while (IsAutonomous() && IsEnabled()) {
			GetWatchdog().Feed();

			//	Start the Angle to go until it is at the top
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "while loop");
			dsLCD->UpdateLCD();

			shooterControl->ShooterAngle(1);

			//	run motors from ShooterControl
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "angle set");
			dsLCD->UpdateLCD();

			shooterControl->SetShooterMotors(fireSpeed);

			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "set motors");
			dsLCD->UpdateLCD();
			//	check if angle has reached desired
			if (shooterControl->getAngle() == 1.0) {
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "if angle");
				fireNow = false;
				if (fireRate.Get() > waitTime) {
					waitTime = 3.0; // change wait time to fire rate
					fireNow = true;
					fireRate.Reset();
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "fire");
					dsLCD->UpdateLCD();
				}
				pneumaticsControl.autoFire(fireNow);
			}
		}
	}

};

START_ROBOT_CLASS(Tim2013)
;

