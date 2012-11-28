
#ifndef LUNACYSHOOTERMOTORS_H
#define LUNACYSHOOTERMOTORS_H

#include "WPILib.h"
#include "ShooterMotors.h"

class LunacyShooterMotors:public ShooterMotors {
public:
	LunacyShooterMotors();
	void setTopShooterMotorSpeed(float speed){shooterTopMotor->Set(speed);}
	void setLiftMotorSpeed(float speed){liftMotor->Set(speed);}
	void setSweeperMotorSpeed(float speed){sweeperMotor->Set(speed);}
private:
	Jaguar* shooterTopMotor;
	Victor* liftMotor;
	Victor* sweeperMotor;
};

#endif 

