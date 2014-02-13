#include "ShooterControl.h"
#include "XboxController.h"
#include "Math.h"
#include "PneumaticsControl.h"
#include "WPILib.h"
#include "PIDControlSubClass.h"

//Defined Constants
#define BALLGRABBERMOTOR5 5
#define BALLGRABBERMOTOR6 6
#define BALLMOTOR5SPEED -0.3
#define BALLMOTOR6SPEED -0.3
#define UPPERSHOOTER 7
#define LOWERSHOOTER 8
#define UPPERSHOOTER2 9
#define LOWERSHOOTER2 10
#define UPPERLIMITSWITCH 8
#define LOWERLIMITSWITCH 7
#define STOPPEDSPEED 0.0
#define FIRESPEED 0.7
#define LOADINGSPEED 0.4 
#define ARMINGSPEED -0.1
#define SHOOTINGSPEED 0.5
#define READYTOFIRELIMIT 100
#define RIGHT 5000
#define HOME 0
#define READYTOFIRE 45
#define FIRE 310
#define PIDFIRE 650
#define Kp 0.005
#define	Ki 0.000002
#define	Kd 0.0003

static ShooterControl *shootercontrol = NULL;
ShooterControl *ShooterControl::getInstance() {
	if (shootercontrol == NULL) {
		shootercontrol = new ShooterControl();
	}
	return shootercontrol;
}

ShooterControl::ShooterControl() {
	xbox = XboxController::getInstance();
	shooterEncoder = new Encoder(10, 9, true, Encoder::k1X); //change 3rd param. to false for production robot
	dsLCD = DriverStationLCD::GetInstance();
	pneumaticsControl = PneumaticsControl::getInstance();
	BallGrabberMotor5 = new Talon(BALLGRABBERMOTOR5);
	BallGrabberMotor6 = new Talon(BALLGRABBERMOTOR6);
	upperShooter = new Talon(UPPERSHOOTER);
	lowerShooter = new Talon(LOWERSHOOTER);
	upperShooter2 = new Talon(UPPERSHOOTER2);
	lowerShooter2 = new Talon(LOWERSHOOTER2);
	pIDControlOutput = new PIDControlSubClass(upperShooter, lowerShooter, upperShooter2, lowerShooter2 );
	controller
			= new PIDController(Kp, Ki, Kd, shooterEncoder, pIDControlOutput);
	counter = 0;
	limitCount = 0;
	upperLimit = new DigitalInput(UPPERLIMITSWITCH);
	lowerLimit = new DigitalInput(LOWERLIMITSWITCH);
	five = new DigitalInput(4);
	maxValue = 0;
	loadingBall = false;
	
	
}

void ShooterControl::initialize() {

	shooterEncoder->Reset();
	shooterEncoder->Start();
	shooterEncoder->SetDistancePerPulse(1);
	shooterEncoder->SetPIDSourceParameter(Encoder::kDistance);
	controller->SetPercentTolerance(85);
	controller->SetContinuous();
	fireState = Init;
	controller->Disable();
	shooterTimer.Stop();
	shooterTimer.Reset();
	shooterTimer.Start();
	previousTime = 0.0;
	
}
double ShooterControl::downRampProfile (double Time){
	//slope is count per second
	//NOT DONE
	LUKASreturn Time * -0.5 + FIRE;
}
/*If B is Pressed, Depending on the PistonState, Retracts or Fires Pistons
 *If A is Held and Pistons are Fired, BallMotorSpeeds are Set to a Value
 *If Y is Held and Pistons are Fired, BallMotorSpeeds are Set to reverse
 */
void ShooterControl::ballGrabber() {
	bool aHeld = xbox->isAPressed();
	bool bPress = xbox->isBPressed();
	bool yHeld = xbox->isYPressed();

	if (bPress) {
		pneumaticsControl->ballGrabberToggle();
	}

	//	if (pneumaticsControl->ballGrabberIsExtended()) {
	if (aHeld) {
		if (BallGrabberMotor5->Get() == STOPPEDSPEED) {
			BallGrabberMotor5->Set(BALLMOTOR5SPEED);
			BallGrabberMotor6->Set(BALLMOTOR5SPEED);
		} else {
			BallGrabberMotor5->Set(STOPPEDSPEED);
			BallGrabberMotor6->Set(STOPPEDSPEED);
		}
	}
	/*
	 } else {
	 BallGrabberMotor5->Set(STOPPEDSPEED);
	 BallGrabberMotor6->Set(STOPPEDSPEED);
	 }
	 }
	 */
	if (yHeld) {
		//if (pneumaticsControl->ballGrabberIsExtended()) {
		if (BallGrabberMotor5->Get() == STOPPEDSPEED) {
			BallGrabberMotor5->Set(-BALLMOTOR5SPEED);
			BallGrabberMotor6->Set(-BALLMOTOR5SPEED);
		} else {
			BallGrabberMotor5->Set(STOPPEDSPEED);
			BallGrabberMotor6->Set(STOPPEDSPEED);
		}
		/*
		 } else {
		 BallGrabberMotor5->Set(STOPPEDSPEED);
		 BallGrabberMotor6->Set(STOPPEDSPEED);
		 }
		 */
	}

	switch (fireState) {
	case Init:
		//extend ball grabber
		//set both feeders to stop
		pneumaticsControl->ballGrabberExtend();
		BallGrabberMotor5->Set(STOPPEDSPEED);
		BallGrabberMotor6->Set(STOPPEDSPEED);
		break;
	case Arming:
		//extend ball grabber
		//set feeders to 0.4(variable?)
		pneumaticsControl->ballGrabberExtend();
		BallGrabberMotor5->Set(-BALLMOTOR5SPEED);
		BallGrabberMotor6->Set(-BALLMOTOR5SPEED);
		break;
	case ReadyToFire:
	case Fired:
	case Firing:
		//stop feeders
		BallGrabberMotor5->Set(STOPPEDSPEED);
		BallGrabberMotor6->Set(STOPPEDSPEED);
		break;

	}

	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "BG: %i ",
			pneumaticsControl->ballGrabberIsExtended());
	dsLCD->UpdateLCD();

}

bool ShooterControl::canIFire() {
	return pneumaticsControl->ballGrabberIsExtended();
}

void ShooterControl::PIDShooter() {
	bool isRTHeld = xbox->isRightTriggerHeld();
	bool isLTHeld = xbox->isLeftTriggerHeld();
	bool isUpperLimit = upperLimit->Get() == 0;
	bool isLowerLimit = lowerLimit->Get() == 0;

	int count = shooterEncoder->Get();
	//bool isLowerLimit = abs(count) < 10;
	switch (fireState) {
	case Init:
		if (isLowerLimit) {
			//upperShooter->Set(STOPPEDSPEED);
			//lowerShooter->Set(STOPPEDSPEED);
			pIDControlOutput->PIDWrite(STOPPEDSPEED);
			shooterEncoder->Reset();
			controller->Enable();
			controller->SetSetpoint(READYTOFIRE);
			fireState = ReadyToFire;
		} else {
			if (canIFire()) {
				//UpperShooter->Set(ARMINGSPEED);
				//LowerShooter->Set(ARMINGSPEED);
				pIDControlOutput->PIDWrite(ARMINGSPEED);
			}
		}

		break;

	case Arming:
		//arming
		//controller->SetSetpoint(HOME);
		if (isLowerLimit) {
			//UpperShooter->Set(STOPPEDSPEED);
			//LowerShooter->Set(STOPPEDSPEED);
			pIDControlOutput->PIDWrite(STOPPEDSPEED);
			shooterEncoder->Reset();
			controller->Enable();
			controller->SetSetpoint(HOME);
			//fireState = ReadyToFire;
		} else {
			//UpperShooter->Set(ARMINGSPEED);
			//LowerShooter->Set(ARMINGSPEED);
			pIDControlOutput->PIDWrite(ARMINGSPEED);
		}
		if (!isLTHeld) {
			//UpperShooter->Set(STOPPEDSPEED);
			//LowerShooter->Set(STOPPEDSPEED);
			pIDControlOutput->PIDWrite(STOPPEDSPEED);
			controller->Enable();
			controller->SetSetpoint(READYTOFIRE);
			fireState = ReadyToFire;
		}

		break;

	case ReadyToFire:
		//ready to fire. Lift up until a value is reached
		/*
		 * if(controller->Get() < (READYTOFIRE - 50)){ //spin ball grabber motors to help move ball to readyToFire position
		 BallGrabberMotor5->Set(BALLMOTOR5SPEED);
		 BallGrabberMotor6->Set(BALLMOTOR5SPEED);
		 }
		 else{
		 BallGrabberMotor5->Set(STOPPEDSPEED);
		 BallGrabberMotor6->Set(STOPPEDSPEED);
		 }
		 controller->SetSetpoint(READYTOFIRE);
		 */
		if ((count > READYTOFIRE - 10) && (count < READYTOFIRE + 10)) {
			if (isRTHeld && canIFire()) {
				fireState = Firing;
				controller->Disable();
				//UpperShooter->Set(SHOOTINGSPEED);
				//LowerShooter->Set(SHOOTINGSPEED);
				pIDControlOutput->PIDWrite(SHOOTINGSPEED);
				//immediatly opperate as changing states
				//controller->SetSetpoint(FIRE);
			}
		}
		if (isLTHeld) {
			fireState = Arming;
			controller->Disable();
			//UpperShooter->Set(ARMINGSPEED);
			//LowerShooter->Set(ARMINGSPEED);
			pIDControlOutput->PIDWrite(ARMINGSPEED);

		}

		break;

	case Firing:
		//firing
		if (!isRTHeld) {
			//UpperShooter->Set(STOPPEDSPEED);
			//LowerShooter->Set(STOPPEDSPEED);
			pIDControlOutput->PIDWrite(STOPPEDSPEED);
			if (canIFire()) {
				controller->Enable();
				controller->SetSetpoint(READYTOFIRE);
				fireState = ReadyToFire;
			} else {
				controller->Enable();
				controller->SetSetpoint(shooterEncoder->Get());
				fireState = Fired;
			}
		} else {
			if (isUpperLimit || shooterEncoder->Get() > FIRE) {
				controller->Enable();
				controller->SetSetpoint(FIRE);
				fireState = Fired;
			} else {
				//UpperShooter->Set(SHOOTINGSPEED);
				//LowerShooter->Set(SHOOTINGSPEED);
				pIDControlOutput->PIDWrite(SHOOTINGSPEED);
			}
		}

		break;

	case Fired:
		if (!isRTHeld && canIFire()) {
			controller->SetSetpoint(READYTOFIRE);
			fireState = ReadyToFire;
		}
		break;
	}

	dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "S: %s C: %i",
			GetStateString(), count);

}

char*ShooterControl::GetStateString() {
	switch (fireState) {
	case Arming:
		return "Arming";

	case ReadyToFire:
		return "ReadyToFire";

	case Firing:
		return "Firing";

	case Init:
		return "Init";

	case Fired:
		return "Fired";

	default:
		return "";

	}
}

void ShooterControl::ManualShoot() {
	float rightValue = 0.0;
	rightValue = xbox->getAxisRightY();
	//bool isUpperLimit = upperLimit->Get();
	//bool isLowerLimit = lowerLimit->Get();
	/*
	 if (isLowerLimit){
	 UpperShooter->Set(0.0);
	 LowerShooter->Set(0.0);
	 }
	 else if(isUpperLimit){
	 UpperShooter->Set(0.0);
	 LowerShooter->Set(0.0);
	 }else{
	 UpperShooter->Set((rightValue / 3.0));
	 LowerShooter->Set((rightValue / 3.0));
	 }
	 */
	//UpperShooter->Set((rightValue / 3.0));
	//LowerShooter->Set((rightValue / 3.0));
	pIDControlOutput->PIDWrite(rightValue / 2.0);
	float count = shooterEncoder->Get();
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Count x: %f", count);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "MotorSpeed %f",
			rightValue);
	dsLCD->UpdateLCD();
}

void ShooterControl::run() {
	ballGrabber();
	//ManualShoot();
	PIDShooter();
}

