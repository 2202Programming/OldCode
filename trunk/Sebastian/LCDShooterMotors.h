#ifndef LCDSHOOTERMOTORS_H
#define LCDSHOOTERMOTORS_H

#include "WPILib.h"
#include "ShooterMotors.h"

class LCDShooterMotors: public ShooterMotors {
public:
	LCDShooterMotors();
	void setTopShooterMotorSpeed(float speed);
	void setBotShooterMotorSpeed(float speed);
	void setLiftMotorSpeed(float speed);
	void setFeederMotorSpeed(float speed);
	void setSweeperMotorSpeed(float speed);

private:
	DriverStationLCD *dsLCD;

};

#endif 

