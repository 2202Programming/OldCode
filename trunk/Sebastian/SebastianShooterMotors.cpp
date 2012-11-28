#include "SebastianShooterMotors.h"

SebastianShooterMotors::SebastianShooterMotors() {
	shooterTopMotor = new Jaguar(3);
	shooterBotMotor = new Jaguar(4);
	liftMotor = new Jaguar(8);
	feederMotor = new Victor(7);
}

