#ifndef CONTROLLER_H
#define CONTROLLER_H 

class Controller {
public:
	virtual bool isLeftJogPressed()= 0;
	virtual bool isRightJogPressed()= 0;
	virtual bool isUpJogPressed()= 0;
	virtual bool isDownJogPressed()= 0;
	virtual bool isAPressed()= 0;
	virtual bool isBPressed()= 0;
	virtual bool isXPressed()= 0;
	virtual bool isYPressed()= 0;
	virtual bool isLBumperPressed()= 0;
	virtual bool isRBumperPressed()= 0;
	virtual bool isBackPressed()= 0;
	virtual bool isStartPressed()= 0;
	virtual bool isL3Pressed()= 0;
	virtual bool isR3Pressed()= 0;
	virtual bool isAHeld()= 0;
	virtual bool isBHeld()= 0;
	virtual bool isXHeld()= 0;
	virtual bool isYHeld()= 0;
	virtual bool isLBumperHeld()= 0;
	virtual bool isRBumperHeld()= 0;
	virtual bool isBackHeld()= 0;
	virtual bool isStartHeld()= 0;
	virtual bool isL3Held()= 0;
	virtual bool isR3Held()= 0;
	virtual bool isRightTriggerHeld()= 0;
	virtual bool isLeftTriggerHeld()= 0;
	virtual float getAxisRightX()= 0;
	virtual float getAxisRightY()= 0;
	virtual float getAxisLeftX()= 0;
	virtual float getAxisLeftY()= 0;
	virtual float getAxisTrigger()= 0;
	virtual bool JogEnabled()= 0;
	virtual ~Controller() {
	}
private:
};

#endif 
