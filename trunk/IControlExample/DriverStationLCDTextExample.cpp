
#include "WPILib.h"
#include "IControl.h"
#include "LCDControl.h"

/**
 * This example shows how you can write text to the LCD on the driver station.
 */ 
class DriverStationLCDTextExample : public SimpleRobot
{
	IControl *control;
	DriverStationLCD *dsLCD;
public:
	DriverStationLCDTextExample()
	{
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		control = new LCDControl();
		dsLCD->UpdateLCD();
	}

	/*void RobotMain()
	{
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Hello World");
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 11, "Time: %4.1f", GetClock());
		dsLCD->UpdateLCD();

		Wait(0.1);
	}*/
	/**
		 * Drive left & right motors for 2 seconds, enabled by a jumper (jumper
		 * must be in for autonomous to operate).
		 */
		void Autonomous()
		{
			
			control->initializeAuto();
			while (IsAutonomous())
			{
				control->runAuto();
				dsLCD->UpdateLCD();
				Wait(0.005);
			}
		}

		/**
		 * Runs the motors under driver control with either tank or arcade steering selected
		 * by a jumper in DS Digin 0. Also an arm will operate based on a joystick Y-axis. 
		 */
		void OperatorControl()
		{
			control->initialize();
			while (IsOperatorControl())
			{
				control->run();
				dsLCD->UpdateLCD();
				Wait(0.005);
			}
		}
	
};

START_ROBOT_CLASS(DriverStationLCDTextExample);

