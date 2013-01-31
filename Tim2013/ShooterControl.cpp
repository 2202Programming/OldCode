/* This is the CPP file for Shooter Control 
 * Current Version:
 * Hold Right Trigger for Shooting
 * Press X to toggle lift motor
 * Press LBumper to increment speed
 * */

#include "ShooterControl.h"
#include "XboxController.h"

#define SHOOTERMOTORPORT1 5
#define SHOOTERMOTORPORT2 6
#define SHOOTERSPEEDINCREMENT .1
#define SHOOTERSPEEDINCREMENTRESETPT 0.0

ShooterControl::ShooterControl() {
	xbox = XboxController::getInstance();
	dsLCD = DriverStationLCD::GetInstance();
	shooterMotor1 = new Jaguar(SHOOTERMOTORPORT1);
	shooterMotor2 = new Jaguar(SHOOTERMOTORPORT2);

	liftMotor = new Jaguar(8); //temporary, please get rid of me
	lShooterSpeed = 0.0;
	rShooterSpeed = 0.0;
}

void ShooterControl::initialize() {
}

void ShooterControl::initializeAutonomous() {
}
//this method is an experiment for shooting calibration, will be removed from final program
//this method cycles though the shooter speeds
void ShooterControl::ExperimentShooter() {

	bool isLBumperPressed = xbox->isLBumperPressed();
	if (isLBumperPressed) {
		lShooterSpeed = speedIncriment(lShooterSpeed);
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "left speed: %f",
				lShooterSpeed);
		dsLCD->UpdateLCD();
	}
	shooterMotor1->Set(lShooterSpeed);

	bool isRBumperPressed = xbox->isRBumperPressed();
	if (isRBumperPressed) {
		rShooterSpeed = speedIncriment(rShooterSpeed);
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "right speed: %f",
				rShooterSpeed);
		dsLCD->UpdateLCD();
	}
	shooterMotor2->Set(rShooterSpeed);
}
/*
 * ShootMotor:
 * When the Right trigger is pressed, the Shooter motor turns on. 
 * Must be held.
 */
float ShooterControl::speedIncriment(float speed) {
		speed += SHOOTERSPEEDINCREMENT;
		if (speed > 1.01) {
			speed = 0.0;
		}
	
	return speed;
}
void ShooterControl::ShootMotor() {

	/*
	 * Left bumper = increase speed
	 */
	// speed incriment using left bumper.
	/*
	 * Right trigger = turn on shooter motor
	 */
	float isTriggerPressed = xbox->getAxisTrigger();
	float shootSpeed = 0.0;
	if (isTriggerPressed <= -.01) {
		shootSpeed = speedIncriment(shootSpeed);
	}
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "speed: %f", shootSpeed);
	dsLCD->UpdateLCD();
	shooterMotor1->Set(-shootSpeed);
	shooterMotor2->Set(-shootSpeed);

}
/*
 * LiftMotor: This is for SEBASTIAN only. GET RID OF FOR TIM.
 * When X is pressed, the lift is turned on. Toggles.
 * 
 */
void ShooterControl::LiftMotor() {
	//temporary, please get rid of me
	if (xbox->isXHeld()) {
		liftMotor->Set(1);
	} else {
		liftMotor->Set(0);
	}
}

//press right trigger to shoot
void ShooterControl::run() {
	//ShootMotor();
	ExperimentShooter();
	//LiftMotor();
}

