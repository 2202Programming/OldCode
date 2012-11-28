#include "LCDShooterMotors.h"


LCDShooterMotors::LCDShooterMotors() {
	dsLCD = DriverStationLCD::GetInstance();

	dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "LCDShooterMotors");

}

void LCDShooterMotors::setTopShooterMotorSpeed(float speed) {
//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "TShoot %8f",speed);
}
void LCDShooterMotors::setBotShooterMotorSpeed(float speed) {
	//dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "BShoot %8f",speed);
}
void LCDShooterMotors::setLiftMotorSpeed(float speed) {
	//dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "Lift %8f",speed);
}
void LCDShooterMotors::setFeederMotorSpeed(float speed) {
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Feed %8f",speed);
}
void LCDShooterMotors::setSweeperMotorSpeed(float speed) {
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "Sweep %8f",speed);
}
