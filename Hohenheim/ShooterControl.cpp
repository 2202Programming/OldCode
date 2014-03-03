#include "ShooterControl.h"
#include "XboxController.h"
#include "Math.h"
#include "PneumaticsControl.h"
#include "WPILib.h"
#include "PIDControlSubClass.h"

//Defined Constants
#define BALLGRABBERMOTOR5 5
#define BALLGRABBERMOTOR6 6
#define BALLMOTOR5SPEED -0.5
#define BALLMOTOR6SPEED -0.5
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
#define RIGHT 5000
#define HOME 5
#define ARMING 0
#define READYTOFIRE 30 // 90
#define FIRE 260
#define TRUSS 250
#define PASS 200 
#define PIDTOLERANCE 5.0
#define RETRACTCPS  175.0 //for down ramp profile
#define SHOOTCPS 750.0 //for the shoot ramp
#define PASSCPS 200.0 //rate of the passing ramp
#define TRUSSCPS 1300.0 //rate of the truss throw ramp 
#define LOADCPS 100.0
#define Kp 0.012
#define	Ki 0.00075
#define	Kd 0.006

static ShooterControl *shootercontrol = NULL;
ShooterControl *ShooterControl::getInstance() {
	if (shootercontrol == NULL) {
		shootercontrol = new ShooterControl();
	}
	return shootercontrol;
}

ShooterControl::ShooterControl() {
	xbox = XboxController::getInstance();
	shooterEncoder = new Encoder(10, 9, true, Encoder::k4X); //change 3rd param. to false for production robot
	dsLCD = DriverStationLCD::GetInstance();
	pneumaticsControl = PneumaticsControl::getInstance();
	BallGrabberMotor5 = new Talon(BALLGRABBERMOTOR5);
	BallGrabberMotor6 = new Talon(BALLGRABBERMOTOR6);
	upperShooter = new Talon(UPPERSHOOTER);
	lowerShooter = new Talon(LOWERSHOOTER);
	upperShooter2 = new Talon(UPPERSHOOTER2);
	lowerShooter2 = new Talon(LOWERSHOOTER2);
	pIDControlOutput = new PIDControlSubClass(upperShooter, lowerShooter,
			upperShooter2, lowerShooter2);
	controller
			= new PIDController(Kp, Ki, Kd, shooterEncoder, pIDControlOutput);
	counter = 0;
	limitCount = 0;
	upperLimit = new DigitalInput(UPPERLIMITSWITCH);
	lowerLimit = new DigitalInput(LOWERLIMITSWITCH);
	five = new DigitalInput(4);
	maxValue = 0;
	loadingBall = false;
	LED1 = new Relay(1, Relay::kForwardOnly);
	LED2 = new Relay(3, Relay::kForwardOnly);
	LED3 = new Relay(5, Relay::kForwardOnly);
	autoShot = false; //turns to true as soon as it is shot in autonomous
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
	previousTime = shooterTimer.Get();
	//LED1->Set(Relay::kOn);
	//LED2->Set(Relay::kOn);
	//LED3->Set(Relay::kOn);
	lightCounter = 0;
}

void ShooterControl::initializeAuto() {
	shooterTimer.Stop();
	shooterTimer.Reset();
	shooterEncoder->Reset();
	shooterEncoder->Start();
	shooterEncoder->SetDistancePerPulse(1);
	shooterEncoder->SetPIDSourceParameter(Encoder::kDistance);
	controller->SetPercentTolerance(85);
	controller->SetContinuous();
	controller->Disable();
	autoFireState = AutoInit;
	doneAutoFired = false;
	previousTime = 0.0;
	cummulativeTime = 0.0;
}

void ShooterControl::autoLoad(bool on) {
	if (on) {
		BallGrabberMotor5->Set(-BALLMOTOR5SPEED * 0.75);
		BallGrabberMotor6->Set(-BALLMOTOR5SPEED * 0.75);
	} else {
		BallGrabberMotor5->Set(STOPPEDSPEED);
		BallGrabberMotor6->Set(STOPPEDSPEED);
	}
}

void ShooterControl::autoShoot() {
	bool isUpperLimit = upperLimit->Get() == 0;
	bool isLowerLimit = lowerLimit->Get() == 0;
	double timeChange = (shooterTimer.Get() - previousTime);
	previousTime = shooterTimer.Get();

	int count = shooterEncoder->Get();
	if (!doneAutoFired) {
		switch (autoFireState) {
		case AutoInit:	
			if (isLowerLimit) {
				//Set Home Position and switches states to autoready, starts shooterTimer
				pIDControlOutput->PIDWrite(STOPPEDSPEED);
				shooterEncoder->Reset();
				controller->Enable();
				controller->SetSetpoint(HOME);
				autoFireState = AutoReady;
				shooterTimer.Start();
				cummulativeTime = 0.0;
			} else {
				if (canIFire()) {
					// Drop Down the Arm If Loader is extended
					pIDControlOutput->PIDWrite(ARMINGSPEED);
				}
			}
			break;
		case AutoReady:
			if ((count > READYTOFIRE - PIDTOLERANCE) && (count < READYTOFIRE
					+ PIDTOLERANCE)) {
				controller->SetSetpoint(READYTOFIRE);
				cummulativeTime += timeChange;
				if (cummulativeTime >= 2.5) { //waits 2.5 seconds once arm is at readyToFire
					if (canIFire()) {
						autoFireState = AutoFire;
					}
				}
			} else { //Brings arm up to ready to fire position
//				cummulativeTime = 0.0;
				double positionChange = loadRampProfile(timeChange);
				double newSetpoint = controller->GetSetpoint() + positionChange;
				if (newSetpoint >= READYTOFIRE) {
					newSetpoint = READYTOFIRE;
				}
				controller->SetSetpoint(newSetpoint);
			}
			break;
		case AutoFire:
			if (isUpperLimit || shooterEncoder->Get() > FIRE) {
				controller->SetSetpoint(FIRE);
				doneAutoFired = true;
			} else {
				double countChange = shootRampProfile(timeChange);
				double newSetpoint = controller->GetSetpoint() + countChange;
				if (newSetpoint >= FIRE) {
					newSetpoint = FIRE;
				}
				controller->SetSetpoint(newSetpoint);
			}
			break;
		}

	}
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "AS: %s",
			GetAutoStateString());
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "EncoderC: %i",
			shooterEncoder->Get());
	dsLCD->UpdateLCD();
}

void ShooterControl::feed(bool toggleMotor) {
	if (toggleMotor) {
		BallGrabberMotor5->Set(-BALLMOTOR5SPEED);
		BallGrabberMotor6->Set(-BALLMOTOR6SPEED);
	} else {
		BallGrabberMotor5->Set(STOPPEDSPEED);
		BallGrabberMotor6->Set(STOPPEDSPEED);
	}

}

void ShooterControl::toggleColor() {
	bool isStartPressed = xbox->isStartPressed();
	//red=1, blue 3, 2=unused
	if (isStartPressed) {
		if (lightCounter == 0) {
			LED1->Set(Relay::kOn);
			LED3->Set(Relay::kOff);
			lightCounter++;
		} else if (lightCounter == 1) {
			LED1->Set(Relay::kOff);
			LED3->Set(Relay::kOn);
			lightCounter++;
		} else {
			LED1->Set(Relay::kOn);
			LED3->Set(Relay::kOn);
			lightCounter = 0;
		}
	}

	/*
	 if (lightCounter == 0) {
	 //0-red
	 LED1->Set(Relay::kOn);
	 LED3->Set(Relay::kOff);
	 } else {
	 if (lightCounter == 1) {
	 //1-blue
	 LED1->Set(Relay::kOff);
	 LED3->Set(Relay::kOn);
	 } else {
	 //2-both
	 LED1->Set(Relay::kOn);
	 LED3->Set(Relay::kOn);
	 }
	 }
	 if (isStartPressed) {
	 if (lightCounter = 2) {
	 lightCounter = 0;
	 } else {
	 lightCounter++;
	 }
	 }
	 */
	//LED1->Set(Relay::kOn);
	//LED2->Set(Relay::kOn);
	//LED3->Set(Relay::kOn);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Counter: %i",
			lightCounter);
	dsLCD->UpdateLCD();

}

char*ShooterControl::GetAutoStateString() {

	switch (autoFireState) {
	case AutoInit:
		return "AutoInit";
	case AutoReady:
		return "AutoReady";
	case AutoFire:
		return "AutoFire";
	default:
		return "";
	}
}

bool ShooterControl::doneAutoFire() {
	return doneAutoFired;
}

double ShooterControl::downRampProfile(double timeChange) {
	return (timeChange * -RETRACTCPS);
}

double ShooterControl::shootRampProfile(double timeChange) {
	return (timeChange * SHOOTCPS);
}

double ShooterControl::passRampProfile(double timeChange) {
	return (timeChange * PASSCPS);
}

double ShooterControl::trussRampProfile(double timeChange) {
	return (timeChange * TRUSSCPS);
}

double ShooterControl::loadRampProfile(double timeChange) {
	return (timeChange * LOADCPS);
}

void ShooterControl::ballGrabber() {
	//bool aHeld = xbox->isAPressed();
	bool bPress = xbox->isBPressed();
	bool aHeld = xbox->isAHeld();
	//bool yHeld = xbox->isYHeld();

	if (bPress) {
		pneumaticsControl->ballGrabberToggle();
	}

	double ballGrabberOutput = STOPPEDSPEED;
	switch (fireState) {
	case Home:
		pneumaticsControl->ballGrabberRetract();
		break;
	case Init:
		pneumaticsControl->ballGrabberExtend();
		break;
	case Arming:
		pneumaticsControl->ballGrabberExtend();
		if (pneumaticsControl->ballGrabberIsExtended() && aHeld) {
			ballGrabberOutput = -BALLMOTOR5SPEED;
		} else {
			ballGrabberOutput = STOPPEDSPEED;
		}
		break;
	case Retracting:
	case Fired:
	case Firing:
	case ReadyToFire:
		pneumaticsControl->ballGrabberExtend();
		if (pneumaticsControl->ballGrabberIsExtended() && aHeld) {
			ballGrabberOutput = -BALLMOTOR5SPEED;
		} else {
			ballGrabberOutput = STOPPEDSPEED;
		}

		break;
	case Passing:
		pneumaticsControl->ballGrabberRetract();
		if (!pneumaticsControl->ballGrabberIsExtended()) {
			ballGrabberOutput = BALLMOTOR5SPEED;
		}
	default:
		break;
	}

	BallGrabberMotor5->Set(ballGrabberOutput);
	BallGrabberMotor6->Set(ballGrabberOutput);
//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "BG: %i ",
//			pneumaticsControl->ballGrabberIsExtended());
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
	bool isRBHeld = xbox->isRBumperHeld();
	bool isXHeld = xbox->isXHeld();

	//for ramping
	double timeChange = (shooterTimer.Get() - previousTime);
	previousTime = shooterTimer.Get();
	int count = shooterEncoder->Get();

	switch (fireState) {
	case Init: //Starts The Robot In This State
		if (isLowerLimit) {
			pIDControlOutput->PIDWrite(STOPPEDSPEED);
			shooterEncoder->Reset();
			controller->Enable();
			controller->SetSetpoint(HOME);
			fireState = Home;
		} else {
			if (canIFire()) {
				pIDControlOutput->PIDWrite(ARMINGSPEED);
			}
		}
		break;
	case Home: //Default State For Shooter Arm
		//controller->SetSetpoint(HOME);
		if (isRBHeld) {
			fireState = ReadyToFire;
		}
		if (isLTHeld) {
			fireState = Arming;
		}
		if (isRTHeld) {
			fireState = Passing;
		}
		break;
	case Arming: //If LTHeld 
		if (isLowerLimit) {
			pIDControlOutput->PIDWrite(STOPPEDSPEED);
			shooterEncoder->Reset();
			controller->Enable();
			controller->SetSetpoint(ARMING);
		} else {
			pIDControlOutput->PIDWrite(ARMINGSPEED);
		}
		if (!isLTHeld) {
			pIDControlOutput->PIDWrite(STOPPEDSPEED);
			controller->Enable();
			controller->SetSetpoint(HOME);
			fireState = Home;
		}
		break;

	case ReadyToFire:
		//ready to fire. Lift up until a value is reached
		if (!isRBHeld) {
			controller->SetSetpoint(shooterEncoder->Get());
			fireState = Retracting;
		} else if ((count > READYTOFIRE - 3) && (count < READYTOFIRE + 3)) {
			if (isRTHeld && canIFire()) {
				fireState = Firing;
			}
			if (isXHeld) {
				fireState = TrussShot;
			}
		} else {
			if (pneumaticsControl->ballGrabberIsExtended()) {
				double positionChange = loadRampProfile(timeChange);
				double newSetpoint = controller->GetSetpoint() + positionChange;
				if (newSetpoint >= READYTOFIRE) {
					newSetpoint = READYTOFIRE;
				}
				controller->SetSetpoint(newSetpoint);
			}
		}
		break;

	case TrussShot:
		if (!isXHeld) {
			if (canIFire()) {
				fireState = Retracting;
				controller->SetSetpoint(shooterEncoder->Get());
			} else {
				controller->SetSetpoint(shooterEncoder->Get());
				fireState = Fired;
			}
		} else {
			if (isUpperLimit || shooterEncoder->Get() >= TRUSS) {
				controller->SetSetpoint(TRUSS);
				fireState = Fired;
			} else {
				double countChange = trussRampProfile(timeChange);
				double newSetpoint = controller->GetSetpoint() + countChange;
				if (newSetpoint >= TRUSS) {
					newSetpoint = TRUSS;
				}
				controller->SetSetpoint(newSetpoint);
			}
		}
		break;

	case Passing:
		if (!isRTHeld) {
			fireState = Home;
		}
		break;

	case Firing:
		//firing
		if (!isRTHeld) {
			if (canIFire()) {
				fireState = Retracting;
				controller->SetSetpoint(shooterEncoder->Get());
			} else {
				controller->SetSetpoint(shooterEncoder->Get());
				fireState = Fired;
			}
		} else {
			if (isUpperLimit || shooterEncoder->Get() >= FIRE) {
				controller->SetSetpoint(FIRE);
				fireState = Fired;
			} else {
				//pIDControlOutput->PIDWrite(SHOOTINGSPEED);
				double countChange = shootRampProfile(timeChange);
				double newSetpoint = controller->GetSetpoint() + countChange;
				if (newSetpoint >= FIRE) {
					newSetpoint = FIRE;
				}
				controller->SetSetpoint(newSetpoint);
			}
		}

		break;

	case Fired:
		if (!isRTHeld && canIFire()) {
			//controller->SetSetpoint(READYTOFIRE);
			//fireState = ReadyToFire;
			fireState = Retracting;
		}
		break;

	case Retracting:
		if (this->shooterEncoder->Get() <= HOME + 5) {
			controller->SetSetpoint(HOME);
			fireState = Home;
		}
		double positionChange = downRampProfile(timeChange);
		double newSetpoint = controller->GetSetpoint() + positionChange;
		if (newSetpoint <= HOME) {
			newSetpoint = HOME;
		}
		controller->SetSetpoint(newSetpoint);
		break;
	default:
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
	case Retracting:
		return "Retracting";
	case Home:
		return "Home";
	case Passing:
		return "Passing";
	case TrussShot:
		return "TrussShot";
	default:
		return "";
	}

}

void ShooterControl::ManualShoot() {
	float rightValue = 0.0;
	rightValue = xbox->getAxisRightY();
	pIDControlOutput->PIDWrite(rightValue / 2.0);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "MotorSpeed %f",
			rightValue);
	dsLCD->UpdateLCD();
}

void ShooterControl::run() {
	//ManualShoot();
	ballGrabber();
	PIDShooter();
	toggleColor();
}

