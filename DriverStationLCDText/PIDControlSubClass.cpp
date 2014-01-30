#include "PIDControlSubClass.h"

PIDControlSubClass::PIDControlSubClass(UINT32 channel1, UINT32 channel2) :
	motor1(channel1), motor2(channel2) {

}
void PIDControlSubClass::PIDWrite(float output) {
	//float invertOutput = output * -1.0;
	motor1.PIDWrite(output);
	motor2.PIDWrite(output);
}
