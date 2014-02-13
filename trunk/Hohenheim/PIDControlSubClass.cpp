#include "PIDControlSubClass.h"

PIDControlSubClass::PIDControlSubClass(PIDOutput* motorIn1,
		PIDOutput* motorIn2, PIDOutput* motorIn3, PIDOutput* motorIn4) {
	motor1 = motorIn1;
	motor2 = motorIn2;
	motor3 = motorIn3;
	motor4 = motorIn4;

}
void PIDControlSubClass::PIDWrite(float output) {
	motor1->PIDWrite(output);
	motor2->PIDWrite(output);
	motor3->PIDWrite(output);
	motor4->PIDWrite(output);

}
