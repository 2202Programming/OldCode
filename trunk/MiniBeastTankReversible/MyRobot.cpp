#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class RobotDemo: public SimpleRobot {

	
	RobotDrive *myRobot; // robot drive system
	Joystick *rightStick; // joystick 1 (arcade stick or right tank stick)
	Joystick *leftStick; // joystick 2 (tank left stick)
	DriverStation *ds; // driver station object
	DriverStationLCD *dsLCD; // driver station message screen
	
public:
	RobotDemo(void) {
		ds = DriverStation::GetInstance();
		myRobot = new RobotDrive(1, 2); // create robot drive base
		rightStick = new Joystick(1); // create the joysticks
		leftStick = new Joystick(1);
		rightStick->SetAxisChannel(Joystick::kXAxis, 4);
		rightStick->SetAxisChannel(Joystick::kYAxis, 5);
		//Update the motors at least every 100ms.
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "MiniTankReversible");
		dsLCD->UpdateLCD();
		myRobot->SetExpiration(0.1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)

	{/*
		myRobot->SetSafetyEnabled(false);
		myRobot->Drive(0.5, 0.0); // drive forwards half speed
		Wait(2.0); //    for 2 seconds
		myRobot->Drive(0.0, 0.0); // stop robot
		*/
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void) {
		bool isForward = 1;
		int key7pressed = 0;

		
		while (IsOperatorControl()) {
			dsLCD->UpdateLCD();
	//     if back button(7) is pressed, toggle forward and backward drive for robot
			if (leftStick->GetRawButton(7)) {
				// This debounces the button
				key7pressed++; 
				// Button must be read as pressed 60 times, then toggle
				if (key7pressed == 40) {
					isForward = !isForward;
				}
			} else {
				// If button not pressed, counter reset.
				key7pressed = 0;
			}
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Direction:");
			if (!isForward) {
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3,
						"Towards camera.");
			} else {
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3,
						"Away from camera.");
			}
			dsLCD->UpdateLCD();
			float leftStickY = leftStick->GetY();
			float rightStickY = rightStick->GetY();
			// reverse y values for joysticks
			if (isForward) {
				leftStickY = -leftStick->GetY();
				rightStickY = -rightStick->GetY();
			}
				
			myRobot->TankDrive(leftStickY ,rightStickY ); // drive with tank style
			
			Wait(0.005);
		}
	}
};

START_ROBOT_CLASS(RobotDemo)
;

