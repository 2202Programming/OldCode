#include "WPILib.h"
#include "XboxController.h"
/**
 * this is a test for four wheel driving
 */
class RobotDemo: public SimpleRobot {
	RobotDrive myRobot; // robot drive system
//	Joystick rightstick;
//	Joystick leftstick;
	DriverStationLCD *dsLCD;
	XboxController xbox;

public:
	RobotDemo(void):
		myRobot(3,4,1,2), // These are initialized so that the motors are on the right sides as the controller
//		rightstick(1), leftstick(4)
		xbox(1)
	
	
	{
		
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Four Wheel");
		dsLCD->UpdateLCD();
		myRobot.SetExpiration(0.1);

	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void) {
		myRobot.SetSafetyEnabled(false);
		myRobot.Drive(0.5, 0.0); 	// drive forwards half speed
		Wait(2.0); //    for 2 seconds
		myRobot.Drive(0.0, 0.0); 	// stop robot
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void) {
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl()) {
//			myRobot.TankDrive(leftstick, rightstick); // drive with tank style
			myRobot.TankDrive(xbox.getAxisLeftY(), xbox.getAxisRightY());
			dsLCD->Clear();
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "left: %f", xbox.getAxisLeftY());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "right: %f", xbox.getAxisRightY());
			dsLCD->UpdateLCD();
			Wait(0.005); // wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(RobotDemo)
;

