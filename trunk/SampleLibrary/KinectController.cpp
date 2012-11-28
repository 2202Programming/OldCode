#include "WPILib.h"
#include "KinectController.h"
#include <cmath>
/* These are the button mappings to the raw values on the kinect controller*/

static KinectController *kinect = NULL;
KinectController *KinectController::getInstance() {
	if (kinect == NULL) {
		kinect = new KinectController();
	}
	return kinect;
}
#define HEAD_LEFT 1
#define HEAD_RIGHT 2
#define RIGHTLEG_RIGHT 3
#define LEFTLEG_LEFT 4
#define RIGHTLEG_FORWARD 5
#define RIGHTLEG_BACK 6
#define LEFTLEG_FORWARD 7
#define LEFTLEG_BACK 8

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
#define JOG_DEBOUNCE 10
// Number of consecutive hits to count as pressed. 
/* The constructor is fed the number for initializing the Joysticks*/

KinectController::KinectController() :
	lstick(1), rstick(2) {
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
	leftJog = 0;
	upJog = 0;
	downJog = 0;
	rightJog = 0;
	rightTrigger = 0;
	leftTrigger = 0;

}
bool KinectController::isLeftTriggerHeld() {

	return false;
}
bool KinectController::isRightTriggerHeld() {

	DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "raw%6i",
			rstick.GetRawButton(LEFTLEG_FORWARD));
	return isButtonHeld(rightTrigger, rstick.GetRawButton(LEFTLEG_FORWARD));
}


bool KinectController::isBPressed() {
	//return isButtonPressed(b, rstick.GetRawButton(HEAD_LEFT));
	return false;
}

bool KinectController::isLBumperPressed() {
	return false;
}

bool KinectController::isRBumperPressed() {
	return false;
}
float KinectController::getAxisRightY() {
	return (-1.0) * rstick.GetRawAxis(2);
}
float KinectController::getAxisLeftY() {
	return (-1.0) * lstick.GetRawAxis(2);
}
bool KinectController::JogEnabled() {
	return false;
}
/* Returns true if activated*/
bool KinectController::isLeftJogPressed() {
	return false;
}
bool KinectController::isUpJogPressed() {
	return false;
}
bool KinectController::isDownJogPressed() {
	return false;
}
bool KinectController::isRightJogPressed() {
	return false;
}
bool KinectController::isAPressed() {
	return false;
}

bool KinectController::isXPressed() {
	return false;
}

bool KinectController::isYPressed() {
	return false;
}

bool KinectController::isBackPressed() {
	return false;
}

bool KinectController::isStartPressed() {
	return false;
}

bool KinectController::isL3Pressed() {
	return false;
}

bool KinectController::isR3Pressed() {
	return false;
}

bool KinectController::isAHeld() {
	return false;
}

bool KinectController::isBHeld() {
	return false;
}

bool KinectController::isXHeld() {
	return false;
}

bool KinectController::isYHeld() {
	return false;
}

bool KinectController::isLBumperHeld() {
	return false;
}

bool KinectController::isRBumperHeld() {
	return false;
}

bool KinectController::isBackHeld() {
	return false;
}

bool KinectController::isStartHeld() {
	return false;
}
bool KinectController::isL3Held() {
	return false;
}

bool KinectController::isR3Held() {
	return false;
}
/* The Raw Axis values are inverted so that they make sense. Up is positive now. */
float KinectController::getAxisRightX() {
	return (0.0);
}

float KinectController::getAxisLeftX() {
	return (0.0);
}

/* Remeber that the right and left trigger make up one axis total. */
float KinectController::getAxisTrigger() {
	return 0.0;
}
bool KinectController::isButtonPressed(int &counter, bool isRawPressed,
		int debouncedCount) {
	if (isRawPressed) {
		// This debounces the button
		counter++;
		// Button must be recounterd counters pressed 60 times, then toggle
		if (counter == debouncedCount) {
			return true;
		}
	} else {
		// If button not pressed, counter reset.
		counter = 0;
	}
	return false;
}
bool KinectController::isButtonPressed(int &counter, bool rawValue) {
	return isButtonPressed(counter, rawValue, KINECTDEBOUNCER);
}
bool KinectController::isButtonHeld(int &counter, bool isRawPressed) {
	return isButtonHeld(counter, isRawPressed, KINECTDEBOUNCER);
}
bool KinectController::isButtonHeld(int &counter, bool rawValue,
		int debounceCount) {

	if (rawValue) {
		// This debounces the button
		counter++;
		// Button must be recounterd counters pressed 60 times, then toggle
		if (counter > debounceCount) {
			return true;
		}
	} else {
		// If button not pressed, counter reset.
		//counter = 0;
	}
	return false;
}

