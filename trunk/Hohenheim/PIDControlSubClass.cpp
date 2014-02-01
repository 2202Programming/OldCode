#include "PIDControlSubClass.h"

PIDControlSubClass::PIDControlSubClass(PIDOutput* motorIn1, PIDOutput* motorIn2){
motor1 = motorIn1;
motor2 = motorIn2;
}
void PIDControlSubClass::PIDWrite (float output){
motor1->PIDWrite(output);
motor2->PIDWrite(output);

}
