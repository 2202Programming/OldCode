#include "WPILib.h"
#include "XboxController.h"

#define LEFTWHEELS_CHANNEL 1
#define RIGHTWHEELS_CHANNEL 2
#define TOPMOTOR_CHANNEL 3
#define BOTTOMMOTOR_CHANNEL 4
#define SPEEDCHANGE 0.05

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class RobotDemo: public SimpleRobot {
	RobotDrive myRobot; // robot drive system

	Jaguar topMotor;
	Jaguar bottomMotor;
	XboxController xboxController; //robot  
	DriverStationLCD *dsLCD;
public:
	RobotDemo(void) :

				myRobot(LEFTWHEELS_CHANNEL, RIGHTWHEELS_CHANNEL), // these must be initialized in the same order
				topMotor(TOPMOTOR_CHANNEL), bottomMotor(BOTTOMMOTOR_CHANNEL),
				xboxController(1) // as they are declared above.
	{
		myRobot.SetExpiration(0.1);
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "testBed");
		dsLCD->UpdateLCD();
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void) {
		//		myRobot.SetSafetyEnabled(false);
		//		myRobot.Drive(0.5, 0.0); // drive forwards half speed
		//		Wait(2.0); //    for 2 seconds
		//		myRobot.Drive(0.0, 0.0); // stop robot
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void) {
		myRobot.SetSafetyEnabled(true);
		float topSpeed = 0.0;
		float bottomSpeed = 0.0;

		while (IsOperatorControl()) {
			if (xboxController.getLBumper()) {
				topSpeed += SPEEDCHANGE;
			}
			if (xboxController.getRBumper()) {
				bottomSpeed += SPEEDCHANGE;
			}
			if (xboxController.getBack()) {
				topSpeed -= SPEEDCHANGE;
			}
			if (xboxController.getStart()) {
				bottomSpeed -= SPEEDCHANGE;
			}
			if (topSpeed >= 1) {
				topSpeed = 1.0;
			}
			if (topSpeed <= 0) {
				topSpeed = 0.0;
			}
			if (bottomSpeed >= 1) {
				bottomSpeed = 1.0;
			}
			if (bottomSpeed <= 0) {
				bottomSpeed = 0.0;
			}

			myRobot.TankDrive(xboxController.getAxisLeftY(), xboxController.getAxisRightY());
			
			topMotor.Set(topSpeed);
			bottomMotor.Set(bottomSpeed);
			
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "topSpeed: %f",
					topSpeed);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "bottomSpeed: %f",
					bottomSpeed);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "lefty: %f",
					xboxController.getAxisLeftY());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "righty: %f",
					xboxController.getAxisRightY());
			dsLCD->UpdateLCD();
			//			myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
			Wait(0.005); // wait for a motor update time

		}
	}
};

START_ROBOT_CLASS(RobotDemo)
;

