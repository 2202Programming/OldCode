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
	shooterStartSpeed = .4;
	isLoadingPosition = false;
	isFiringPosition = false;
}

void ShooterControl::initialize() {
	Angle = 0.0;
	Stop();
}

//void ShooterControl::initializeAutonomous() {
//	Angle = 0.0;
//}

//this method cycles though the shooter speeds in 4 steps
void ShooterControl::ShooterSeperateCycleSpeed() {
	bool isLBumperPressed = xbox->isLBumperPressed();
	if (isLBumperPressed) {
		Shooter1Speed += SHOOTERSPEEDSTEP;
		if (Shooter1Speed > 1)
			Shooter1Speed = 0;

	}
	bool isRBumperPressed = xbox->isRBumperPressed();

	if (isRBumperPressed) {

		Shooter2Speed += SHOOTERSPEEDSTEP;
		if (Shooter2Speed > 1)
			Shooter2Speed = 0;
	}
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "s1: %f, s2: %f",
			Shooter1Speed, Shooter2Speed);
	dsLCD->UpdateLCD();

	shooterMotor1->Set(Shooter1Speed);
	shooterMotor2->Set(Shooter2Speed);

}
void ShooterControl::Stop() {
	Shooter1Speed = 0;
	Shooter2Speed = 0;
	shooterMotor1->Set(Shooter1Speed);
	shooterMotor2->Set(Shooter2Speed);
}
void ShooterControl::APushStop() {
	bool isAPressed = xbox->isAPressed();
	if (isAPressed) {
		Stop();
		isLoadingPosition = false;
		isFiringPosition = false;

	}
}

void ShooterControl::ShooterCycleBehindSpeed() {

	bool isLBumperPressed = xbox->isLBumperPressed();

	// if left bumper is pressed reduce the speed for both motors
	if (isLBumperPressed) {
		Shooter2Speed = (Shooter2Speed -= SHOOTERSPEEDSTEP) < 0 ? 0
				: Shooter2Speed;
		// if shooter speed is less than shooterstart speed. stop
		if (Shooter2Speed < shooterStartSpeed)
			Shooter2Speed = 0;

		// Both variables are set to the same speed, but we are using two variables incase we want to set two different values.
		// From final code extra variable can be removed
	}

	// if right bumper is pressed increase the speed for both motors
	bool isRBumperPressed = xbox->isRBumperPressed();

	if (isRBumperPressed) {

		Shooter2Speed = (Shooter2Speed += SHOOTERSPEEDSTEP) > 1.0 ? 1
				: Shooter2Speed;

		// if shooter speed is equal to first speed increment (.25) jump to shooter start speed (.4).
		if (Shooter2Speed == SHOOTERSPEEDSTEP)
			Shooter2Speed = shooterStartSpeed;

		// Both variables are set to the same speed, but we are using two variables incase we want to set two different values.
		// From final code extra variable can be removed
	}
	if (Shooter2Speed < .3) {
		Shooter1Speed = Shooter2Speed;
	} else {
		Shooter1Speed = Shooter2Speed - SHOOTERSPEEDSTEPDIFF;
	}

	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "shooterspeed: %f",
			Shooter2Speed);
	dsLCD->UpdateLCD();

	shooterMotor1->Set(Shooter1Speed);
	shooterMotor2->Set(Shooter2Speed);

}

void ShooterControl::ShooterCycleSpeed() {

	bool isLBumperPressed = xbox->isLBumperPressed();

	// if left bumper is pressed reduce the speed for both motors
	if (isLBumperPressed) {
		Shooter1Speed = (Shooter1Speed -= SHOOTERSPEEDSTEP) < 0 ? 0
				: Shooter1Speed;
		// if shooter speed is less than shooterstart speed. stop
		if (Shooter1Speed < shooterStartSpeed)
			Shooter1Speed = 0;

		// Both variables are set to the same speed, but we are using two variables incase we want to set two different values.
		// From final code extra variable can be removed
		Shooter2Speed = Shooter1Speed;
	}

	// if right bumper is pressed increase the speed for both motors
	bool isRBumperPressed = xbox->isRBumperPressed();

	if (isRBumperPressed) {

		Shooter1Speed = (Shooter1Speed += SHOOTERSPEEDSTEP) > 1.0 ? 1
				: Shooter1Speed;

		// if shooter speed is equal to first speed increment (.25) jump to shooter start speed (.4).
		if (Shooter1Speed == SHOOTERSPEEDSTEP)
			Shooter1Speed = shooterStartSpeed;

		// Both variables are set to the same speed, but we are using two variables incase we want to set two different values.
		// From final code extra variable can be removed
		Shooter2Speed = Shooter1Speed;
	}

	shooterMotor1->Set(Shooter1Speed);
	shooterMotor2->Set(Shooter2Speed);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "s1: %f s2:%f",
			shooterMotor1->Get(), shooterMotor2->Get());
	dsLCD->UpdateLCD();
}

bool ShooterControl::isLowestAngle() {
	return !lowerLimit->Get();
}
// this method sets the angle of the shooter using a motor. elevation is increased when right trigger is pressed, and it is decreased
void ShooterControl::ShooterAngle(float angleDirection) {
	//rightAngle = xbox->getAxisRightY();
	bool upperOn = !upperLimit->Get();
	bool lowerOn = !lowerLimit->Get();

	// if we are hitting the limit, cancel the direction so that we don't move the motor
	if (lowerOn) {
		if (angleDirection < 0) {
			angleDirection = 0;
			isLoadingPosition = false;
		}
	}

	if (upperOn) {
		if (angleDirection > 0)
			angleDirection = 0;
		isFiringPosition = false;
	}

	// check for dead zone i.e +- 1. move motor if beyond dead zone
	if (fabs(angleDirection) > .1) {
		AngleMotor->Set(-1 * angleDirection);
		Angle += angleDirection;
	} else {
		AngleMotor->Set(0.0);
	}

	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "U: %i L: %i", upperOn,
			lowerOn);
	dsLCD->UpdateLCD();
}

bool ShooterControl::isRunning() {
	float speed = shooterMotor1->Get();
	if (speed == 0)
		return false;
	else
		return true;

}

void ShooterControl::SetShooterMotors(float speed) {
	shooterMotor1->Set(speed);
	shooterMotor2->Set(speed);
}

float ShooterControl::getAngle() {
	// use this for now
	return maxAngleReached();
}

bool ShooterControl::maxAngleReached() {
	return !upperLimit->Get();
}

//press right trigger to shoot
void ShooterControl::run() {
	APushStop();
	//ShooterSeperateCycleSpeed();float axisY = xbox->getAxisRightY();
	goTo();
	float axisY = xbox->getAxisRightY();
	if (fabs(axisY) > .2) {
		isLoadingPosition = false;
		isFiringPosition = false;
	}
	if (isLoadingPosition)//loading position
	{
		ShooterAngle(SHOOTERLOADINGDIRECTION);
		Shooter1Speed = SHOOTERLOADINGSPEED;
		Shooter2Speed = SHOOTERLOADINGSPEED;
	} else if (isFiringPosition) {
		ShooterAngle(SHOOTERFIRINGDIRECTION);
		Shooter1Speed = SHOOTERFIRINGSPEED;
		Shooter2Speed = SHOOTERFIRINGSPEED;
	} else {

		//ShooterCycleBehindSpeed();
		ShooterAngle(axisY);
		if (xbox->isXPressed()) {
			Shooter2Speed = DEFAULTSHOOTERSPEED;
			Shooter1Speed = DEFAULTSHOOTERSPEED;
		}
	}
	ShooterCycleSpeed();
}
//void ShooterControl::runAutonomous() {
//shooterMotor1->Set(AUTOSPEED);
//shooterMotor2->Set(AUTOSPEED);
//
//}
/*
 * if Y is pressed, goes to loading position, turns motors off
 * If B is pressed, goes to firing position, turns motors on
 */
void ShooterControl::goTo() {
	if (xbox->isYPressed()) {
		isLoadingPosition = true;
	} else if (xbox->isBPressed()) {
		isFiringPosition = true;
	}

}

