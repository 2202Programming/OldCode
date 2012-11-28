#include "LunacyShooterMotors.h"

LunacyShooterMotors::LunacyShooterMotors() {
	shooterTopMotor = new Jaguar(4);
	liftMotor = new Victor(5);
	sweeperMotor = new Victor(6);
}

