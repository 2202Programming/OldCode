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
#define UPPERLIMITSWITCH 7
#define LOWERLIMITSWITCH 8
#define STOPPEDSPEED 0.0
#define FIRESPEED 0.7
#define LOADINGSPEED 0.4 
#define ARMINGSPEED -0.3
#define READYTOFIRELIMIT 100
#define RIGHT 5000
#define HOME 0
#define READYTOFIRE 100
#define FIRE 620
#define PIDFIRE 650
#define Kp 0.003
#define	Ki 0.0
#define	Kd 0.00001

static ShooterControl *shootercontrol = NULL;
ShooterControl *ShooterControl::getInstance() {
	if (shootercontrol == NULL) {
		shootercontrol = new ShooterControl();
	}
	return shootercontrol;
}

ShooterControl::ShooterControl() {
	xbox = XboxController::getInstance();
	shooterEncoder = new Encoder(10, 9, false, Encoder::k1X);
	dsLCD = DriverStationLCD::GetInstance();
	pneumaticsControl = PneumaticsControl::getInstance();
	BallGrabberMotor5 = new Talon(BALLGRABBERMOTOR5);
	BallGrabberMotor6 = new Talon(BALLGRABBERMOTOR6);
	UpperShooter = new Talon(UPPERSHOOTER);
	LowerShooter = new Talon(LOWERSHOOTER);
	//pIDControlOutput = new PIDControlSubClass(UpperShooter, LowerShooter);
	//controller
	//		= new PIDController(Kp, Ki, Kd, shooterEncoder, pIDControlOutput);
	counter = 0;
	twoStageFire = Rest;
	upperLimit = new DigitalInput(UPPERLIMITSWITCH);
	lowerLimit = new DigitalInput(LOWERLIMITSWITCH);
	maxValue = 0;
	loadingBall = false;
}

void ShooterControl::initialize() {

	shooterEncoder->Reset();
	shooterEncoder->SetDistancePerPulse(1);
	shooterEncoder->Start();
	shooterEncoder->SetPIDSourceParameter(Encoder::kDistance);
	//controller->SetPercentTolerance(85);
	//controller->SetContinuous();
	//controller->Enable();
	//controller->SetSetpoint(HOME);
	fireState = Init;

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
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "BG: %i",
			pneumaticsControl->ballGrabberIsExtended());

	if (pneumaticsControl->ballGrabberIsExtended()) {
		if (aHeld) {
			if (BallGrabberMotor5->Get() == STOPPEDSPEED) {
				BallGrabberMotor5->Set(BALLMOTOR5SPEED);
				BallGrabberMotor6->Set(BALLMOTOR5SPEED);
			} else {
				BallGrabberMotor5->Set(STOPPEDSPEED);
				BallGrabberMotor6->Set(STOPPEDSPEED);
			}

		} else {
			BallGrabberMotor5->Set(STOPPEDSPEED);
			BallGrabberMotor6->Set(STOPPEDSPEED);
		}
	}

	if (yHeld) {
		if (pneumaticsControl->ballGrabberIsExtended()) {
			if (BallGrabberMotor5->Get() == STOPPEDSPEED) {
				BallGrabberMotor5->Set(-BALLMOTOR5SPEED);
				BallGrabberMotor6->Set(-BALLMOTOR5SPEED);
			} else {
				BallGrabberMotor5->Set(STOPPEDSPEED);
				BallGrabberMotor6->Set(STOPPEDSPEED);
			}

		} else {
			BallGrabberMotor5->Set(STOPPEDSPEED);
			BallGrabberMotor6->Set(STOPPEDSPEED);
		}
	}

	/*
	 switch (fireState) {
	 case Init:
	 //extend ball grabber
	 //set both feeders to stop
	 pneumaticsControl->ballGrabberToggle();
	 BallGrabberMotor5->Set(STOPPEDSPEED);
	 BallGrabberMotor6->Set(STOPPEDSPEED);
	 break;
	 case Arming:
	 //extend ball grabber
	 //set feeders to 0.4(variable?)
	 pneumaticsControl->ballGrabberToggle();
	 BallGrabberMotor5->Set(-BALLMOTOR5SPEED);
	 BallGrabberMotor6->Set(-BALLMOTOR5SPEED);
	 break;
	 case ReadyToFire:
	 //set feeders to 0.4(variable?)
	 BallGrabberMotor5->Set(-BALLMOTOR5SPEED);
	 BallGrabberMotor6->Set(-BALLMOTOR5SPEED);
	 break;
	 case Firing:
	 //stop feeders
	 BallGrabberMotor5->Set(STOPPEDSPEED);
	 BallGrabberMotor6->Set(STOPPEDSPEED);
	 break;
	 }
	 */
	int count = shooterEncoder->Get();
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Count: %i", count);
}

bool ShooterControl::canIFire() {
	return pneumaticsControl->ballGrabberIsExtended();

}

void ShooterControl::PIDShooter() {
	bool isRTHeld = xbox->isRightTriggerHeld();
	bool isLTHeld = xbox->isLeftTriggerHeld();
	bool isUpperLimit = upperLimit->Get();
	bool isLowerLimit = lowerLimit->Get();
	int count = shooterEncoder->Get();

	switch (fireState) {
	case Init: {
		if (isLowerLimit) {
			UpperShooter->Set(STOPPEDSPEED);
			LowerShooter->Set(STOPPEDSPEED);
			shooterEncoder->Reset();
			controller->Enable();
			controller->SetSetpoint(READYTOFIRE);
			fireState = ReadyToFire;
		} else {
			if (canIFire()) {
				UpperShooter->Set(ARMINGSPEED);
				LowerShooter->Set(ARMINGSPEED);
			}
		}
	}
		break;

	case Arming: {
		//arming
		//controller->SetSetpoint(HOME);
		if (isLowerLimit) {
			UpperShooter->Set(STOPPEDSPEED);
			LowerShooter->Set(STOPPEDSPEED);
			shooterEncoder->Reset();
			controller->Enable();
			controller->SetSetpoint(HOME);
			//fireState = ReadyToFire;
		} else {
			UpperShooter->Set(ARMINGSPEED);
			LowerShooter->Set(ARMINGSPEED);
		}
		if (!isLTHeld) {
			UpperShooter->Set(STOPPEDSPEED);
			LowerShooter->Set(STOPPEDSPEED);
			controller->Enable();
			controller->SetSetpoint(READYTOFIRE);
			fireState = ReadyToFire;
		}
	}
		break;

	case ReadyToFire: {
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

		if (isRTHeld && canIFire()) {
			fireState = Firing;
			//immediatly opperate as changing states
			controller->SetSetpoint(FIRE);
		}
		if (isLTHeld) {
			fireState = Arming;
			controller->Disable();
			UpperShooter->Set(ARMINGSPEED);
			LowerShooter->Set(ARMINGSPEED);
		}
	}
		break;

	case Firing: {
		//firing

		if (isUpperLimit || !isRTHeld) {
			controller->Disable();
			UpperShooter->Set(STOPPEDSPEED);
			LowerShooter->Set(STOPPEDSPEED);
			if (canIFire()) {
				controller->Enable();
				controller->SetSetpoint(READYTOFIRE);
				fireState = ReadyToFire;
			}
		}
	}
		break;
	}

	dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "S: %s C: %i",
			GetStateString(), count);

}

char*ShooterControl::GetStateString() {
	switch (fireState) {
	case Arming: {
		return "Arming";
	}
	case ReadyToFire: {
		return "ReadyToFire";
	}
	case Firing: {
		return "Firing";
	}
	case Init: {
		return "Init";
	}
	default: {
		return "";
	}
	}
}

void ShooterControl::ManualShoot() {
	float rightValue = 0.0;
	rightValue = xbox->getAxisRightY();
	UpperShooter->Set(rightValue);
	LowerShooter->Set(rightValue);

	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "MotorSpeed %f",
			rightValue);
	dsLCD->UpdateLCD();
}

void ShooterControl::run() {
	ballGrabber();
	ManualShoot();
	//PIDShooter();
}

