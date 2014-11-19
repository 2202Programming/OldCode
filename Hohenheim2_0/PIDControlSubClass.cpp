#include "PIDControlSubClass.h"

PIDControlSubClass::PIDControlSubClass(PIDOutput* motorIn1,
PIDOutput* motorIn2, PIDOutput* motorIn3, PIDOutput* motorIn4) {
	PIDOverideEnabled = false;
	motor1 = motorIn1;
	motor2 = motorIn2;
	motor3 = motorIn3;
	motor4 = motorIn4;
	hasTwoMotors = false;
}
void PIDControlSubClass::PIDWrite(float write) {
	if (PIDOverideEnabled) {
		write = PIDOverideValue;
	}
	motor1->PIDWrite(write);
	motor2->PIDWrite(write);
	if (!hasTwoMotors) {
		motor3->PIDWrite(write);
		motor4->PIDWrite(write);

	}
}
PIDControlSubClass::PIDControlSubClass(PIDOutput* motorIn1, PIDOutput* motorIn2) {
	PIDOverideEnabled = false;
	motor1 = motorIn1;
	motor2 = motorIn2;
	hasTwoMotors = true;

}
//PIDControlSubClass::PIDControlSubClass(PIDOutput* motorIn1,
//		PIDOutput* motorIn2, bool invertOutput) {
//	this->invertOutput = invertOutput;
//	motor1 = motorIn1;
//	motor2 = motorIn2;
//	hasTwoMotors = true;
//}
