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

	AngleMotor = new Victor(SHOOTERANGLEMOTORPORT);

	shooterMotor1 = new Jaguar(SHOOTERMOTORPORT1);
	shooterMotor2 = new Jaguar(SHOOTERMOTORPORT2);
	Shooter1Speed = 0.0;
	Shooter2Speed = 0.0;
	upperLimit = new DigitalInput(UPPERLIMITPORT);
	lowerLimit = new DigitalInput(LOWERLIMITPORT);
	
}

void ShooterControl::initialize() {
	Angle = 0.0;
}

//void ShooterControl::initializeAutonomous() {
//	Angle = 0.0;
//}

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
void ShooterControl::ShooterAngle(float angleDirection) {
	//rightAngle = xbox->getAxisRightY();
	bool upperOn = !upperLimit->Get();
	bool lowerOn = lowerLimit->Get();
	
	// if we are hitting the limit, cancel the direction so that we don't move the motor
	if (lowerOn) {
		if (angleDirection < 0) {
			angleDirection = 0;
		}
	}
	
	if (upperOn) {
		if (angleDirection > 0)
			angleDirection = 0;
	}
	
	// check for dead zone i.e +- 1. move motor if beyond dead zone
	if (fabs(angleDirection) > .1) {
		AngleMotor->Set(-1 * angleDirection);
		Angle += angleDirection;
	} else {
		AngleMotor->Set(0.0);
	}
	
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "U: %i L: %i",
				upperOn, lowerOn);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "Angle: %f", Angle);
		dsLCD->UpdateLCD();
}

bool ShooterControl::isRunning() {
	float speed = shooterMotor1->Get();
	if (speed == 0)
		return false;
	else
		return true;

}


void ShooterControl::SetShooterMotors(float speed){
	shooterMotor1->Set(speed);
	shooterMotor2->Set(speed);	
}

float ShooterControl::getAngle()
{
	// use this for now
	return maxAngleReached();
}

bool ShooterControl::maxAngleReached()
{
	return !upperLimit->Get();
}

//press right trigger to shoot
void ShooterControl::run() {
	ShooterCycleSpeed();
	ShooterAngle(xbox->getAxisRightY());

}
//void ShooterControl::runAutonomous() {
//shooterMotor1->Set(AUTOSPEED);
//shooterMotor2->Set(AUTOSPEED);
//
//}

