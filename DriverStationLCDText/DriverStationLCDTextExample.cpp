#include "WPILib.h"
#include "XboxController.h"
#include "PIDControlSubClass.h"
#define LEFT 0
#define RIGHT 650
/**
 * This example shows how you can write text to the LCD on the driver station.
 */
class DriverStationLCDTextExample: public SimpleRobot {
	Encoder* input;
	PIDControlSubClass* output;
	PIDController* controller;
	DriverStationLCD *dsLCD;
	XboxController *xbox;
	bool isLeft;
	float Kp;
	float Ki;
	float Kd;

public:
	DriverStationLCDTextExample() {
		xbox = XboxController::getInstance();
		dsLCD = DriverStationLCD::GetInstance();

	}
	void initialize() {
		Kp = 0.0010;
		Ki = 0.0001;
		Kd = 0.0;		
		input = new Encoder(10, 9, true, Encoder::k1X);
		input->Reset();
		output = new PIDControlSubClass(8,9);
		input->SetDistancePerPulse(1);
		input->Start();
		
		input->SetPIDSourceParameter(Encoder::kDistance);
		controller = new PIDController(Kp, Ki, Kd, input, output);
		controller->SetPercentTolerance(85);
		controller->SetContinuous();
		controller->Enable();
		controller->SetSetpoint(LEFT);
		isLeft = true;
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1,
				"Kp: %f Ki: %f Kd: %f", Kp, Ki, Kd);
		dsLCD->UpdateLCD();
	}
	void run() {

		bool xPress = xbox->isXPressed();
		int count = input->Get();
		
		if (xPress) {
		
			if (isLeft) {
				isLeft = false;
				controller->SetSetpoint(LEFT);
			} else {
				isLeft = true;
				controller->SetSetpoint(RIGHT);
			}

		}
		
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "State: %s",
				(isLeft ? "left" : "right"));
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "Count: %i", count);
		dsLCD->UpdateLCD();

	}

	void OperatorControl(void) {
		initialize();
		while (IsOperatorControl() && IsEnabled()) {
			
			run();
			dsLCD->UpdateLCD();
			Wait(0.005); // wait for a motor update time
		}
		controller->Disable(); 
		input->Stop();
		delete controller;
		delete input;
	}

};

START_ROBOT_CLASS(DriverStationLCDTextExample)
;

