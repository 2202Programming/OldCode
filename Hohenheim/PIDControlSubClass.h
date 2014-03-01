#ifndef PIDCONTROLSUBCLASS_H
#define PIDCONTROLSUBCLASS_H

#include "WPILib.h"

class PIDControlSubClass: public PIDOutput {
public:
	virtual ~PIDControlSubClass(){
	}
	PIDControlSubClass(PIDOutput*, PIDOutput*, PIDOutput*, PIDOutput*);
	PIDControlSubClass(PIDOutput*, PIDOutput*);
//	PIDControlSubClass(PIDOutput*, PIDOutput*, bool invertOutput);
	void PIDWrite (float output);
private:
PIDOutput* motor1;
PIDOutput* motor2;
PIDOutput* motor3;
PIDOutput* motor4;
bool hasTwoMotors;
bool invertOutput;

};
#endif
