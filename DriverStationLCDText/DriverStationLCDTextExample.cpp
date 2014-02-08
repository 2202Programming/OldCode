#include "WPILib.h"
#include "XboxController.h"
#include "PIDControlSubClass.h"
#define LEFT 150
#define RIGHT 600
#define FRONTRIGHTMOTOR 1
#define BACKRIGHTMOTOR 2
#define FRONTLEFTMOTOR 3
#define BACKLEFTMOTOR 4
/**
 * This example shows how you can write text to the LCD on the driver station.
 */
class DriverStationLCDTextExample: public SimpleRobot {
	Encoder* input;
	//PIDControlSubClass* output;
	//PIDController* controller;
	DriverStationLCD *dsLCD;
	XboxController *xbox;
	RobotDrive myRobot;
	Talon* left;
	Talon* right;
	bool isLeft;
	float Kp;
	float Ki;
	float Kd;

public:
	DriverStationLCDTextExample() :
		myRobot(FRONTLEFTMOTOR, BACKLEFTMOTOR, FRONTRIGHTMOTOR, BACKRIGHTMOTOR) {
		xbox = XboxController::getInstance();
		dsLCD = DriverStationLCD::GetInstance();
		 left = new Talon(7);
		 right = new Talon(8);

	}
	void initialize() {
		Kp = 0.003;
		Ki = 0.0;
		Kd = 0.00001;
		
		input = new Encoder(10, 9, false, Encoder::k1X);
		input->Reset();
		//output = new PIDControlSubClass(8, 9);
		input->SetDistancePerPulse(1);
		input->Start();

		input->SetPIDSourceParameter(Encoder::kDistance);
		//controller = new PIDController(Kp, Ki, Kd, input, output);
		//controller->SetPercentTolerance(85);
		//controller->SetContinuous();
		//controller->Enable();
		//controller->SetSetpoint(LEFT);
		isLeft = true;
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1,
				"Kp: %f Ki: %f Kd: %f", Kp, Ki, Kd);
		dsLCD->UpdateLCD();
	}
	void run() {

	//	bool xPress = xbox->isXPressed();
		int count = input->Get();
/*
		if (xPress) {

			if (isLeft) {
				isLeft = false;
				Kp = 0.001;
				Ki = 0.0;
				Kd = 0.00001;
				controller->SetPID(Kp, Ki, Kd);
				controller->SetSetpoint(LEFT);
			} else {
				isLeft = true;
				Kp = 0.003;
				Ki = 0.0;
				Kd = 0.00001;
				controller->SetPID(Kp, Ki, Kd);
				controller->SetSetpoint(RIGHT);
			}

		}
		

		dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "State: %s",
				(isLeft ? "left" : "right"));
				*/
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "Count: %i", count);
		dsLCD->UpdateLCD();

	}

	void Drive() {
		float moveValue = 0.0;
		float rotateValue = 0.0;

		// friction value is added as a constant to motor to make it more responsive to joystick at lower value
		float frictionValue = 0.0;
		float rotateFriction = 0.0;
		float SpeedControl = 1.5;
		moveValue = xbox->getAxisLeftY();
		rotateValue = xbox->getAxisLeftX();

		// if move value is above the dead zone set friction value to .2
		if (moveValue > .1) {
			frictionValue = 0.2;
		} else if (moveValue < -.1) {
			frictionValue = -.2;
		}
		if (rotateValue > .1) {
			rotateFriction = 0.2;
		} else if (rotateValue < -.1) {
			rotateFriction = -.2;
		}

		myRobot.ArcadeDrive(((moveValue + frictionValue) / SpeedControl),
				(rotateValue + rotateFriction) / SpeedControl);
	}

	
	void Shoot(){
	float rightValue = 0.0;
	rightValue = xbox->getAxisRightY();
	left->Set(rightValue);
	right->Set(rightValue);

	dsLCD->PrintfLine(DriverStationLCD::kUser_Line1,
					"MotorSpeed %f" , rightValue);
			dsLCD->UpdateLCD();
	}
	void OperatorControl(void) {
		//initialize();
		while (IsOperatorControl() && IsEnabled()) {

			//run();
			Shoot();
			Drive();
			dsLCD->UpdateLCD();
			Wait(0.005); // wait for a motor update time
		}
		//controller->Disable();
		//input->Stop();
		//delete controller;
		//delete input;
	}
	
	bool DriverStationLCDTextExample::runAuto() {
		myRobot.ArcadeDrive(.9, .9);
		return (true);
	}

};

START_ROBOT_CLASS(DriverStationLCDTextExample)
;

