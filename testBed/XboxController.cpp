/* This is the CPP file for Xbox Controller */

#include "WPILib.h"
#include "Nivision.h"
#include "XboxController.h"

/* These are the button mappings to the raw values on the xbox controller*/

#define BUTTON_A 1
#define BUTTON_B 2
#define BUTTON_X 3
#define BUTTON_Y 4
#define BUTTON_LB 5
#define BUTTON_RB 6
#define BUTTON_BACK 7
#define BUTTON_START 8
#define BUTTON_L3 9 // Press down the left joystick for L3.
#define BUTTON_R3 10 // Press down the right joystick for R3. David is stupid. 
#define AXIS_RIGHT_X 1
#define AXIS_RIGHT_Y 2
#define AXIS_LEFT_X 4
#define AXIS_LEFT_Y 5 
#define AXIS_TRIGGER 3
#define DEBOUNCER 30 // Number of consecutive hits to count as pressed. 
/* The constructor is fed the number for initializing the Joysticks*/

XboxController::XboxController(int stick) :
	lstick(stick), rstick(stick) {
	rstick.SetAxisChannel(Joystick::kXAxis, 4);
	rstick.SetAxisChannel(Joystick::kYAxis, 5);
	a = 0;
	b = 0;
	x = 0;
	y = 0;
	lb = 0;
	rb = 0;
	back = 0;
	start = 0;
	l3 = 0;
	r3 = 0;
}

/* Returns true if activated*/
bool XboxController::getA() {
	return isButtonPressed(a, rstick.GetRawButton(BUTTON_A));
}

bool XboxController::getB() {
	return isButtonPressed(b, rstick.GetRawButton(BUTTON_B));
}

bool XboxController::getX() {
	return isButtonPressed(x, rstick.GetRawButton(BUTTON_X));
}

bool XboxController::getY() {
	return isButtonPressed(y, rstick.GetRawButton(BUTTON_Y));
}

bool XboxController::getLBumper() {
	return isButtonPressed(lb, rstick.GetRawButton(BUTTON_LB));
}

bool XboxController::getRBumper() {
	return isButtonPressed(rb, rstick.GetRawButton(BUTTON_RB));
}

bool XboxController::getBack() {
	return isButtonPressed(back, rstick.GetRawButton(BUTTON_BACK));
}

bool XboxController::getStart() {
	return isButtonPressed(start, rstick.GetRawButton(BUTTON_START));
}

bool XboxController::getL3() {
	return isButtonPressed(l3, rstick.GetRawButton(BUTTON_L3));
}

bool XboxController::getR3() {
	return isButtonPressed(r3, rstick.GetRawButton(BUTTON_R3));
}
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
bool XboxController::isButtonPressed(int &counter, bool isRawPressed) {
	if (isRawPressed) {
		// This debounces the button
		counter++;
		// Button must be recounterd counters pressed 60 times, then toggle
		if (counter == DEBOUNCER) {
			return true;
		}
	} else {
		// If button not pressed, counter reset.
		counter = 0;
	}
	return false;
}
