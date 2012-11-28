/* This is the Header File for Xbox Controller*/
#include "WPILib.h"
#include "Nivision.h"
#include "Toggler.h"

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
	bool getL3();
	bool getR3();

	float getAxisRightX();
	float getAxisRightY();
	float getAxisLeftX();
	float getAxisLeftY();
	float getAxisTrigger();
private:
	Joystick lstick;
	Joystick rstick;
	int a;
	int b;
	int x;
	int y;
	int lb;
	int rb;
	int back;
	int start;
	int l3;
	int r3;
	bool isButtonPressed (int &counter,bool isPressed);
};

