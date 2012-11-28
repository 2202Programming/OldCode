#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
// If sonar sense at 5 volts gives 9.8 mV/in
class RobotDemo: public SimpleRobot {
	AnalogChannel signal;
	AnalogChannel signalControlVoltage;
	DriverStationLCD *dsLCD;

public:
	RobotDemo(void) :
		signal(3), signalControlVoltage(7) {
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "SonarTest");
		dsLCD->UpdateLCD();

	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void) {

		while (IsOperatorControl()) {
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Voltage: %f",
					signal.GetVoltage());
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "CVoltage: %f",
					signalControlVoltage.GetVoltage());
			dsLCD->UpdateLCD();
			Wait(0.005); // wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(RobotDemo)
;

