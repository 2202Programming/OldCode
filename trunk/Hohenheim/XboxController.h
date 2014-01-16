/* This is the Header File for Xbox Controller*/
#ifndef XBOXCONTROLLER_H
#define XBOXCONTROLLER_H

#define DEBOUNCER 30

#include "WPILib.h"


class XboxController  {
public:
	~XboxController() {}
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
	Joystick* getLeftStick();
	Joystick* getRightStick();
	static XboxController *getInstance();
	bool JogEnabled();
	bool isEndGame();
private:
	XboxController(int stick);
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
	int leftJog;
	int upJog;
	int rightJog;
	int downJog;
	int rightTrigger;
	int leftTrigger;
	int endGame;

	bool isButtonPressed(int &counter, bool rawValue, int debounceCount);
	bool isButtonPressed(int &counter, bool rawValue);
	bool isButtonHeld(int &counter, bool rawValue);
	bool isButtonHeld(int &counter, bool rawValue, int debounceCount);
};

#endif 
