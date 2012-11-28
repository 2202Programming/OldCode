#include "WPILib.h"
#include "XboxController.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	DriverStationLCD *dsLCD;
	RobotDrive myRobot; // robot drive system
	XboxController stick; // only joystick

public:
	RobotDemo(void):
		myRobot(1, 2),	// these must be initialized in the same order
		stick(1)		// as they are declared above.
	{
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "XboxController2");
		dsLCD->UpdateLCD();
		myRobot.SetExpiration(0.1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		myRobot.SetSafetyEnabled(false);
		myRobot.Drive(0.5, 0.0); 	// drive forwards half speed
		Wait(2.0); 				//    for 2 seconds
		myRobot.Drive(0.0, 0.0); 	// stop robot
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			myRobot.ArcadeDrive(stick.getAxisLeftY(), stick.getAxisLeftX()); // drive with arcade style (use left stick)
			dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Move: %f4", stick.getAxisLeftY());
			dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Rotate: %f4", stick.getAxisLeftX());
			dsLCD->UpdateLCD();
			Wait(0.005);				// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

