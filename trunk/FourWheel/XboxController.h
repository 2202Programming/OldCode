/* This is the Header File for Xbox Controller*/
#include "WPILib.h"
#include "Nivision.h"

class XboxController {
public:
	XboxController(int stick);
	bool getA();
	bool getB();
	bool getX();
	bool getY();
	bool getLBumper();
	bool getRBumper();
	bool getBack();
	bool getStart();

	float getAxisRightX();
	float getAxisRightY();
	float getAxisLeftX();
	float getAxisLeftY();
	float getAxisTrigger();
private:
	Joystick lstick;
	Joystick rstick;

};

