#include "ArcadeDriver.h"
#include <Math.h>

//what did the constarcator look like? let me check ... 
//how you gutys doing.	
// pretty good... we learned how to make this stick move.. blah blah blah.
// people talk too fast I cannot type. 
//putted on to the floor, move tables away, and you connect this guy to the hub . 

ArcadeDriver::ArcadeDriver(Joystick* stick, Jaguar* lMotor, Jaguar* rMotor) {
	m_Stick=stick;
	m_RightMotor=rMotor;
	m_LeftMotor=lMotor;
	m_Converter=3.1415926/180.0;
}
ArcadeDriver::~ArcadeDriver() {
}
void ArcadeDriver::Drive() {
	//here is where we do nitty gritty
	//gothum says some stuff with big words.
	float xAxis=m_Stick->GetRawAxis(1); // greater than sign <== . 
	float yAxis=m_Stick->GetRawAxis(2); // gothum is very bossy :c .
	float tolerance=0.1;
	if (fabs(xAxis)< tolerance)
		xAxis=0.0;
	if (fabs(yAxis)< tolerance)
		yAxis=0.0;
	if (yAxis>tolerance)

	{
		//moving backwards (in time).
		if (xAxis > tolerance) {
			//moving to right
			m_LeftMotor->SetSpeed(yAxis * -1.0);
			m_RightMotor->SetSpeed(xAxis); //left is a (-) xAxis; Right is a (+) xAxis; Up is (-) yAxis; Down is (+) yAxis
		} else if (xAxis < -tolerance) {
			//moving to left
			m_LeftMotor->SetSpeed(-xAxis);
			m_RightMotor->SetSpeed(yAxis);

		} else {
			//moving straight back
			m_LeftMotor->SetSpeed(yAxis);
			m_RightMotor->SetSpeed(-yAxis);

		}
	} else if (yAxis < -tolerance) {
		//moving forward (in time) or spinning.
		if (xAxis > tolerance) {
			//moving to right
			m_LeftMotor->SetSpeed(-xAxis);
			m_RightMotor->SetSpeed(-xAxis);
		} else if (xAxis < -tolerance) {
			//moving to left
			m_LeftMotor->SetSpeed(xAxis * -1.0);
			m_RightMotor->SetSpeed(xAxis * -1.0);
		} else {
			//moving foward
			m_LeftMotor->SetSpeed(yAxis); //LeftMotor Forward is (-); LeftMotor backward is (+)
			m_RightMotor->SetSpeed(yAxis * -1.0); // RightMotor forward is (+); RightMotor backward is (-)

		}

	} else {
		m_LeftMotor->SetSpeed(0.0);
		m_RightMotor->SetSpeed(0.0);
	}
}

float ArcadeDriver::GetStickAngle() {
	float angle;
	float tolerance=0.15;
	float x=m_Stick->GetRawAxis(1);
	float y=m_Stick->GetRawAxis(2);

	if (fabs(x)< tolerance)
		x=0.0;
	if (fabs(y)< tolerance)
		y=0.0;
	if (y==0.0) {
		if (x>0.0)
			angle=270.0;
		else
			angle=90.0;
	} else {
		angle=180.0 + float(atan2 (double(x), double(y)))/m_Converter;
		/*if (y>0.0)
		 angle+=180.0;
		 else 
		 {
		 if (x>0.0)
		 angle+=180.0;
		 else
		 angle+=180.0;
		 }*/
	}
	return angle;
}

void ArcadeDriver::Drive2() {
	float oneMotorAngle=60.0;
	float backwardsActiveAngle=60.0;
	float backwardsOneMotorAngle=45.0;
	float angle;
	float speed;
	float tolerance=0.15;
	float x=m_Stick->GetRawAxis(1);
	float y=m_Stick->GetRawAxis(2);
	float leftSpeed = 0.0;
	float rightSpeed = 0.0;

	if (fabs(x)< tolerance)
		x=0.0;
	if (fabs(y)< tolerance)
		y=0.0;
	angle=GetStickAngle();
	speed=float (sqrt(double (x*x)+ double (y*y)));

	if (angle<oneMotorAngle) {
		//Move forward to the left
		//Right Motor forward
		//Left Motor forward then to stop
		leftSpeed = -speed*(oneMotorAngle - angle)/oneMotorAngle; //LeftMotor Forward is (-); LeftMotor backward is (+)
		rightSpeed = speed; // RightMotor forward is (+); RightMotor backward is (-)

	} else if (angle<=90.0) {
		leftSpeed = speed*(angle-oneMotorAngle)/(90.0-oneMotorAngle);
		rightSpeed = speed;
	} else if (angle<180.0-backwardsActiveAngle) {
		leftSpeed = 0.0;
		rightSpeed = 0.0;
	} else if (angle<180.0-backwardsOneMotorAngle) {
		leftSpeed = 0.0;
		rightSpeed = -speed;
	} else if (angle<180.0) {
		leftSpeed = speed*(angle-(180.0-backwardsOneMotorAngle))/backwardsOneMotorAngle;
		rightSpeed = -speed;
	} else if (angle<180.0+backwardsOneMotorAngle) {
		leftSpeed = speed;
		rightSpeed = -speed*(180.0+backwardsOneMotorAngle-angle)/backwardsOneMotorAngle;
	} else if (angle<180.0+backwardsActiveAngle) {
		leftSpeed = speed;
		rightSpeed = 0.0;
	} else if (angle<270.0) {
		leftSpeed = 0.0;
		rightSpeed = 0.0;
	} else if (angle<360-oneMotorAngle) {
		leftSpeed = -speed;
		rightSpeed = -speed*((360.0-oneMotorAngle)-angle)/(90.0-oneMotorAngle);
	} else {
		leftSpeed = -speed;
		rightSpeed = speed*(angle-(360.0-oneMotorAngle))/oneMotorAngle;
	}
	
	if (m_Stick->GetRawAxis(3) > 0.8)
	{
		//Pull left trigger, get slower and more precise movement
		leftSpeed /= 3;
		rightSpeed /= 3;
	}

	m_LeftMotor->SetSpeed(leftSpeed);	//Left motor inverted since first writing this function
	m_RightMotor->SetSpeed(rightSpeed * -1.0);
}

