
#ifndef SHOOTERMOTORS_H
#define SHOOTERMOTORS_H

class ShooterMotors {
public:
	ShooterMotors(){};
	virtual void setTopShooterMotorSpeed(float speed){};
	virtual void setBotShooterMotorSpeed(float speed){};
	virtual void setLiftMotorSpeed(float speed){};
	virtual void setFeederMotorSpeed(float speed){};
	virtual void setSweeperMotorSpeed(float speed){};
private:
	
};

#endif 

