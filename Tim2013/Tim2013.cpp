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

class Tim2013: public SimpleRobot {

	DriverStationLCD *dsLCD;
	DriveControl driveControl;
	ShooterControl shooterControl;

public:
	Tim2013(void) {
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Tim 2013 V 0.0");
		dsLCD->UpdateLCD();
	}

	void Autonomous(void) {
	}

	void OperatorControl(void) {
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Operator control");
		dsLCD->UpdateLCD();

		driveControl.initialize();
		while (IsOperatorControl()) {
			//driveControl.run();
			shooterControl.run();
			Wait(0.005); // wait for a motor update time
		}
	}

};

START_ROBOT_CLASS(Tim2013)
;

