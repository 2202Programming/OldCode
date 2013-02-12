/* This is the CPP file for Shooter Control 
 * Current Version:
 * Hold Right Trigger for Shooting
 * Press X to toggle lift motor
 * Press LBumper to increment speed
 * */

#include "ShooterControl.h"
#include "XboxController.h"

static ShooterControl *shootercontrol = NULL;
ShooterControl *ShooterControl::getInstance() {
	if (shootercontrol == NULL) {
		shootercontrol = new ShooterControl();
	}
	return shootercontrol;
}

ShooterControl::ShooterControl() {

	xbox = XboxController::getInstance();
	dsLCD = DriverStationLCD::GetInstance();

	AngleMotor = new Jaguar(SHOOTERANGLEMOTORPORT);
	AngleMotorRelay = new Relay(1,Relay::kBothDirections);
	
	shooterMotor1 = new Jaguar(SHOOTERMOTORPORT1);
	shooterMotor2 = new Jaguar(SHOOTERMOTORPORT2);
	Shooter1Speed = 0.0;
	Shooter2Speed = 0.0;

	upperLimit = new DigitalInput(UPPERLIMITPORT);
	lowerLimit = new DigitalInput(LOWERLIMITPORT);
}

void ShooterControl::initialize() {

}

void ShooterControl::initializeAutonomous() {
}

//this method cycles though the shooter speeds in 4 steps
void ShooterControl::ShooterCycleSpeed() {

	bool isLBumperPressed = xbox->isLBumperPressed();

	// if left bumper is pressed reduce the speed for both motors
	if (isLBumperPressed) {
		Shooter1Speed = (Shooter1Speed -= SHOOTERSPEEDSTEP) < 0 ? 0
				: Shooter1Speed;

		// Both variables are set to the same speed, but we are using two variables incase we want to set two different values.
		// From final code extra variable can be removed
		Shooter2Speed = Shooter1Speed;
	}

	// if right bumper is pressed increase the speed for both motors
	bool isRBumperPressed = xbox->isRBumperPressed();

	if (isRBumperPressed) {
		Shooter1Speed = (Shooter1Speed += SHOOTERSPEEDSTEP) > 1.0 ? 1
				: Shooter1Speed;

		// Both variables are set to the same speed, but we are using two variables incase we want to set two different values.
		// From final code extra variable can be removed
		Shooter2Speed = Shooter1Speed;
	}

	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "shooterspeed: %f",
			Shooter1Speed);
	dsLCD->UpdateLCD();

	shooterMotor1->Set(Shooter1Speed);
	shooterMotor2->Set(Shooter2Speed);

}
// this method sets the angle of the shooter using a motor. elevation is increased when right trigger is pressed, and it is decreased
//when left trigger is pressed. 

void ShooterControl::ShooterAngle() {

	// Check if left trigger is pressed. if it is then start the lift motor in reverse direction to reduce the elivation of the shooter
	// lowermimit digital input of 1 indicates that there is a room to lower the elivation and we can continue to reduce the angle
	// NOTE: isRightTriggerHeld actually returns left trigger value. 
	bool isXPressed = xbox->isXHeld();

	if (isXPressed && lowerLimit->Get() == 1) {
		AngleMotor->Set(-ANGLEMOTORLIFTSPEED);
	} else {
		AngleMotor->Set(0);
	}

	// same as above except, following lines increse the elivation
	bool isYPressed = xbox->isYHeld();
	if (isYPressed && upperLimit->Get() == 1) {
		AngleMotor->Set(ANGLEMOTORLIFTSPEED);
	} else {
		AngleMotor->Set(0);
	}
}


void ShooterControl::ShooterAngleRelay() {

	// Check if left trigger is pressed. if it is then start the lift motor in reverse direction to reduce the elivation of the shooter
	// lowermimit digital input of 1 indicates that there is a room to lower the elivation and we can continue to reduce the angle
	// NOTE: isRightTriggerHeld actually returns left trigger value. 
	bool isXPressed = xbox->isXHeld();

	if (isXPressed )//&& lowerLimit->Get() == 1) 
		{
		AngleMotorRelay->Set(Relay::kReverse);
		
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "X Pressed");
		dsLCD->UpdateLCD();
	} else {
		AngleMotorRelay->Set(Relay::kOff);
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "         ");
		dsLCD->UpdateLCD();
	}

	// same as above except, following lines increse the elivation
	bool isYPressed = xbox->isYHeld();
	if (isYPressed )//&& upperLimit->Get() == 1) 
	{
		AngleMotorRelay->Set(Relay::kForward);
	} else {
		AngleMotorRelay->Set(Relay::kOff);
	}
	
}
bool ShooterControl::isRunning() {
	float speed = shooterMotor1->Get();
	if (speed == 0)
		return false;
	else
		return true;

}

//press right trigger to shoot
void ShooterControl::run() {
	ShooterCycleSpeed();
	//ShooterAngle();
	ShooterAngleRelay();
}
void ShooterControl::runAutonomous() {
	shooterMotor1->Set(Shooter1Speed);
	shooterMotor2->Set(Shooter2Speed);

}

