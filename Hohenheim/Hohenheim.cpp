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

public:
	Hohenheim(void) {
		driverStation = DriverStation::GetInstance();
		dsLCD = DriverStationLCD::GetInstance();
		pneumaticsControl = PneumaticsControl::getInstance();
		shooterControl = ShooterControl::getInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Hohenheim 2014 V 3.1");
		dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(false);
	}

	void DashBoardInput() {
		int i = 0;
		GetWatchdog().SetEnabled(true);
		while (IsAutonomous() && IsEnabled()) {
			i++;
			GetWatchdog().Feed();
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "%f, %i", driverStation->GetAnalogIn(1), i);
			dsLCD->UpdateLCD();
		}
	}

	void Autonomous(void) {
		DriveThenShootAuto();

	}
	
	void DriveThenShootAuto() {
		//initizlizes all parts of robot
		pneumaticsControl->initialize();
		shooterControl->initializeAuto();
		driveControl.initializeAuto();
		bool destinationPrevious = false;
		bool autoShot = false; //true if autoShoot

		//creates a timer for the ball grabber motors
		Timer feeding;
		bool started = false;
		while (IsAutonomous() && IsEnabled()) {
			GetWatchdog().Feed();
			//drives forward to shooting point
			bool atDestination = destinationPrevious || driveControl.autoPIDDrive2(); //autoDrive returns true when robot reached it's goal
			if (atDestination) {
				// The robot has reached the destination on the floor and is now ready to open and shoot
				if (!started) { 
					started = true;
					destinationPrevious = true;
					//starts feeding-timer controls feeder motors so the ball doesn't get stuck
					feeding.Start();
				}
				
				pneumaticsControl->ballGrabberExtend();
				//waits for feeding to be done
				if (feeding.Get() < 3.0) {
					shooterControl->feed(true);
				}else if(feeding.Get() >= 3.0){
					shooterControl->feed(false);
					feeding.Stop();
				}
				

				if (pneumaticsControl->ballGrabberIsExtended() && !autoShot) {
					shooterControl->autoShoot();
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "The robot is(should be) shooting");
					if (shooterControl->doneAutoFire()) {//works only after shoot is done firing
						autoShot = true;
					}
				} else if(autoShot){//runs only after shoot is done
					dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "AutoMode Finished");
				}
				
			}
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2,"Feeder Time: %f" , feeding.Get());
			dsLCD->UpdateLCD();
		}
	
	}

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

