/* Timothy robot,
 * Soaring disks throw to the goals.
 * Also, climbs. Maybe.
 */
#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"
#include "XboxController.h"
 


class Tim2013 : public SimpleRobot
{
	
	RobotDrive myRobot; // robot drive system
	XboxController *xbox;
	DriverStationLCD *dsLCD;


public:
	Tim2013(void):
		myRobot(1, 2)	// these must be initialized in the same order
	{
		xbox = XboxController::getInstance();
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Tim 2013 is Aliiiiive");
		dsLCD->UpdateLCD();
		myRobot.SetExpiration(0.1);
		myRobot.SetSafetyEnabled(false);
	}

	
	void Autonomous(void)
	{
	
	}

	
	void OperatorControl(void)
	{
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			myRobot.TankDrive(0.0,0.0);
			Wait(0.005);				// wait for a motor update time
		}
	}
	
	
};

START_ROBOT_CLASS(Tim2013);

