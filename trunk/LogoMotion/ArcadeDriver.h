#ifndef __ARCADE_DRIVER_H__
#define __ARCADE_DRIVER_H__
#include "Joystick.h"
#include "WPILib.h"
class ArcadeDriver
{
public:
	ArcadeDriver(Joystick* stick, Jaguar* lMotor, Jaguar* rMotor);
	virtual ~ArcadeDriver();
	void Drive ();
	float GetStickAngle ();
	void Drive2 ();
	private:
		Joystick* m_Stick;
		Jaguar* m_LeftMotor;
		Jaguar* m_RightMotor;
		float m_Converter;
		
};


#endif
