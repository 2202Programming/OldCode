#ifndef PIDCONTROLSUBCLASS_H
#define PIDCONTROLSUBCLASS_H

#include "WPILib.h"

class PIDControlSubClass: public PIDOutput {
public:
	virtual ~PIDControlSubClass(){
	}
	PIDControlSubClass(UINT32 channel1, UINT32 channel2);
	void PIDWrite (float output);
private:
Jaguar motor1;
Jaguar motor2;
};
#endif
