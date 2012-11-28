
#ifndef SEBASTIANSHOOTERMOTORS_H
#define SEBASTIANSHOOTERMOTORS_H

#include "WPILib.h"
#include "ShooterMotors.h"

class SebastianShooterMotors:public ShooterMotors {
public:
	SebastianShooterMotors();
	void setTopShooterMotorSpeed(float speed){shooterTopMotor->Set(speed);}
	void setBotShooterMotorSpeed(float speed){shooterBotMotor->Set(speed);}
	void setLiftMotorSpeed(float speed){liftMotor->Set(speed);}
	void setFeederMotorSpeed(float speed){feederMotor->Set(speed);}
	
private:
	Jaguar* shooterTopMotor;
	Jaguar* shooterBotMotor;
	Jaguar* liftMotor;
	Victor* feederMotor;	
};

#endif 

