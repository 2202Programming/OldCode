/* Timothy robot,
 * Soaring disks throw to the goals.
 * Also, climbs. Maybe.
 */
#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"
#include "XboxController.h"
#include "DriveControl.h"
#include "ShooterControl.h"
#include "PneumaticsControl.h"

class Tim2013: public SimpleRobot {

	DriverStationLCD *dsLCD;
	DriveControl driveControl;
	ShooterControl shooterControl;
	PneumaticsControl pneumaticsControl;

public:
	Tim2013(void) {
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Tim 2013 V 0.1");
		dsLCD->UpdateLCD();
	}

	void Autonomous(void) {
	}

	void OperatorControl(void) {
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Operator control");
		dsLCD->UpdateLCD();

		driveControl.initialize();
		pneumaticsControl.initialize();
		while (IsOperatorControl()) {
			driveControl.run();
			shooterControl.run();
			pneumaticsControl.run();
			Wait(0.005); // wait for a motor update time
		}
	}

};

START_ROBOT_CLASS(Tim2013)
;

