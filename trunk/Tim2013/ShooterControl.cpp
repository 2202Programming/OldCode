/* This is the CPP file for Shooter Control 
 * Current Version:
 * Hold Right Trigger for Shooting
 * Press X to toggle lift motor
 * Press LBumper to increment speed
 * */

#include "ShooterControl.h"
#include "XboxController.h"

#define SHOOTERMOTORPORT1 5
#define SHOOTERMOTORPORT2 6
#define SHOOTERANGLEMOTORPORT 7
#define SHOOTERSPEEDSTEP .1
#define UPPERLIMITPORT 1
#define LOWERLIMITPORT 2
#define ANGLEMOTORLIFTSTEP .30
#define SHOOTERSPEEDINCREMENTRESETPT 0.0

ShooterControl::ShooterControl() {
	xbox = XboxController::getInstance();
	dsLCD = DriverStationLCD::GetInstance();
	shooterMotor1 = new Jaguar(SHOOTERMOTORPORT1);
	shooterMotor2 = new Jaguar(SHOOTERMOTORPORT2);
	AngleMotor = new Jaguar(SHOOTERANGLEMOTORPORT);
	lShooterSpeed = 0.0;
	rShooterSpeed = 0.0;
	upperLimit = new DigitalInput(UPPERLIMITPORT);
	lowerLimit= new DigitalInput(LOWERLIMITPORT);
}

void ShooterControl::initialize(){

}

void ShooterControl::initializeAutonomous() {
}
//this method cycles though the shooter speeds in 4 steps
void ShooterControl::ShooterCycleSpeed() {
	//
	bool isLBumperPressed = xbox->isLBumperPressed();
	if (isLBumperPressed) {
		//lShooterSpeed = speedIncriment(lShooterSpeed); //backmotor
		
		lShooterSpeed = (lShooterSpeed-=SHOOTERSPEEDSTEP ) < 0 ? 0:lShooterSpeed;
		rShooterSpeed = lShooterSpeed;
		
	}

	bool isRBumperPressed = xbox->isRBumperPressed();
	if (isRBumperPressed) {
		lShooterSpeed = (lShooterSpeed+=SHOOTERSPEEDSTEP ) > 1.0 ? 1:lShooterSpeed;
		rShooterSpeed = lShooterSpeed;
		
	}
	
	
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "left speed: %f",
			lShooterSpeed);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "right speed: %f",
			rShooterSpeed);
	dsLCD->UpdateLCD();
	
	shooterMotor1->Set(lShooterSpeed);
	shooterMotor2->Set(rShooterSpeed);

}
// this method sets the angle of the shooter using a motor. when elevation is increased when right trigger is pressed, and and it is decreased
//when left trigger is pressed. 
void ShooterControl::ShooterAngle(){
	bool isRtriggerPressed = xbox->isRightTriggerHeld();
	
	if(isRtriggerPressed && upperLimit->Get() == 0){
		AngleMotor->Set(ANGLEMOTORLIFTSTEP);
	}else{
		AngleMotor->Set(0);
	}
	
	bool isLtriggerPressed = xbox->isLeftTriggerHeld();
		if(isLtriggerPressed && lowerLimit->Get()==0){
			AngleMotor->Set(-ANGLEMOTORLIFTSTEP);
		}else{
			AngleMotor->Set(0);
		}
}

//press right trigger to shoot
void ShooterControl::run() {
	ShooterCycleSpeed();
	//LiftMotor();
}

