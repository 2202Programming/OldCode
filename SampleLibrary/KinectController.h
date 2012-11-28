/* This is the Header File for Xbox Controller*/
#ifndef KINECTCONTROLLER_H
#define KINECTCONTROLLER_H 

#define KINECTDEBOUNCER 10
#include "WPILib.h"
#include "Controller.h"

class KinectController: public Controller {
public:
	~KinectController() {
	}
	bool isLeftJogPressed();
	bool isRightJogPressed();
	bool isUpJogPressed();
	bool isDownJogPressed();
	bool isAPressed();
	bool isBPressed();
	bool isXPressed();
	bool isYPressed();
	bool isLBumperPressed();
	bool isRBumperPressed();
	bool isBackPressed();
	bool isStartPressed();
	bool isL3Pressed();
	bool isR3Pressed();
	bool isAHeld();
	bool isBHeld();
	bool isXHeld();
	bool isYHeld();
	bool isLBumperHeld();
	bool isRBumperHeld();
	bool isBackHeld();
	bool isStartHeld();
	bool isL3Held();
	bool isR3Held();
	bool isRightTriggerHeld();
	bool isLeftTriggerHeld();
	float getAxisRightX();
	float getAxisRightY();
	float getAxisLeftX();
	float getAxisLeftY();
	float getAxisTrigger();
	static KinectController *getInstance();
	bool JogEnabled();

private:
	KinectController();
	KinectStick lstick;
	KinectStick rstick;
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
	int leftJog;
	int upJog;
	int rightJog;
	int downJog;
	int rightTrigger;
	int leftTrigger;
	

	bool isButtonPressed(int &counter, bool rawValue, int debounceCount);
	bool isButtonPressed(int &counter, bool rawValue);
	bool isButtonHeld(int &counter, bool rawValue);
	bool isButtonHeld(int &counter, bool rawValue, int debounceCount);

};

#endif 
