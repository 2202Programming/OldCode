/* This is the CPP file for Xbox Controller */

#include "WPILib.h"
#include "Nivision.h"
#include "XboxController.h"

/* These are the button mappings to the raw values on the xbox controller*/

#define BUTTON_A 1
#define BUTTON_B 2
#define BUTTON_X 3
#define BUTTON_Y 4
#define BUTTON_LEFT_BUMPER 5
#define BUTTON_RIGHT_BUMPER 6
#define BUTTON_BACK 7
#define BUTTON_START 8
#define AXIS_RIGHT_X 1
#define AXIS_RIGHT_Y 2
#define AXIS_LEFT_X 4
#define AXIS_LEFT_Y 5 
#define AXIS_TRIGGER 3

/* The constructor is fed the number for initializing the Joysticks*/

XboxController::XboxController(int stick) :
	lstick(stick), rstick(stick) {
	rstick.SetAxisChannel(Joystick::kXAxis, 4);
	rstick.SetAxisChannel(Joystick::kYAxis, 5);
}

/* Returns true if activated*/
bool XboxController::getA() {
	return rstick.GetRawButton(1);
}

bool XboxController::getB() {
	return rstick.GetRawButton(2);
}

bool XboxController::getX() {
	return rstick.GetRawButton(3);
}

bool XboxController::getY() {
	return rstick.GetRawButton(4);
}

bool XboxController::getLBumper() {
	return rstick.GetRawButton(5);
}

bool XboxController::getRBumper() {
	return rstick.GetRawButton(6);
}

bool XboxController::getBack() {
	return rstick.GetRawButton(7);
}

bool XboxController::getStart() {
return rstick.GetRawButton(8);}

/* The Raw Axis values are inverted so that they make sense. Up is positive now. */
float XboxController::getAxisRightX() {
	return (-1.0) * rstick.GetRawAxis(4);
}

float XboxController::getAxisRightY() {
	return (-1.0) * rstick.GetRawAxis(5);
}

float XboxController::getAxisLeftX() {
	return (-1.0) * lstick.GetRawAxis(1);
}

float XboxController::getAxisLeftY() {
	return (-1.0) * lstick.GetRawAxis(2);
}

/* Remeber that the right and left trigger make up one axis total. */
float XboxController::getAxisTrigger() {
	return rstick.GetRawAxis(3);
}
