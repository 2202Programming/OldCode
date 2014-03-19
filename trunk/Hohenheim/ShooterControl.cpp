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
#define MANUALPIDFIRE 1.00
#define AUTOPIDFIRE 1.00
#define RIGHT 5000
#define HOME 5
#define ARMING 0

#define TRUSSSETUP 15
#define TRUSSPIDSETUP -0.3
#define TRUSSPIDFIRE  1.00
#define TRUSS 110 

#define READYTOFIRE  30 // 40 also Used at Terra Hote // 90 //at Terra Hote it was 30
#define FIRE  225 // 200 //260 Used at Terra Hote 
#define PIDTOLERANCE 5.0
#define RETRACTCPS 250 // 75 //175.0 //for down ramp profile
#define SHOOTCPS 900 //750.0  //  900.0 //750.0 rate for Regionals in indiana//for the shoot ramp
#define LOADCPS 100.0
#define Kp 0.009
#define	Ki 0.0003 // 0.00075
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
	maxEncoderValue = 0;
	twoStageSetupPosition = 0;
	twoStagePidSetup = 0.0;
	twoStageEndPosition = 0;
	twoStagePidFire = 0.0;

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
	pIDControlOutput->PIDOverideDisable();
	//LED1->Set(Relay::kOn); //LED2->Set(Relay::kOn); //LED3->Set(Relay::kOn);
	lightCounter = 0;
}

void ShooterControl::initializeAuto() {
	shooterTimer.Stop();
	shooterTimer.Reset();
	shooterTimer.Start();
	previousTime = shooterTimer.Get();
	shooterEncoder->Reset();
	shooterEncoder->Start();
	shooterEncoder->SetDistancePerPulse(1);
	shooterEncoder->SetPIDSourceParameter(Encoder::kDistance);
	controller->Disable();
	pIDControlOutput->PIDOverideDisable();
	autoFireState = AutoInit;
	doneAutoFired = false;
	cummulativeTime = 0.0;
}

void ShooterControl::autoLoad(bool on) {//Code for Autonomous BallGrabber
	if (on) {
		BallGrabberMotor5->Set(-BALLMOTOR5SPEED * 0.75);
		BallGrabberMotor6->Set(-BALLMOTOR5SPEED * 0.75);
	} else {
		BallGrabberMotor5->Set(STOPPEDSPEED);
		BallGrabberMotor6->Set(STOPPEDSPEED);
	}
}

void ShooterControl::autoShoot() {//Autonomous Shooting Code using Encodere Count and PIDControl
	bool isUpperLimit = upperLimit->Get() == 0;
	bool isLowerLimit = lowerLimit->Get() == 0;
	double timeChange = (shooterTimer.Get() - previousTime);
	previousTime = shooterTimer.Get();

	if (!doneAutoFired) {//A check to only shoot once
		switch (autoFireState) {
		case AutoInit://Puts the arm down until lowerLimit is reached and resets shooterEncoder. Transitions to GoHome.
			if (isLowerLimit) {
				shooterEncoder->Reset();
				controller->Enable();
				controller->SetSetpoint(HOME);
				//pIDControlOutput->PIDWrite(0.3);
				autoFireState = GoHome;
				//				shooterTimer.Start();
			} else {
				if (canIFire()) {// Drop Down the Arm If Loader is extended
					pIDControlOutput->PIDWrite(ARMINGSPEED);
				}
			}
			break;
		case GoHome://If the Encoder Count is Greater Than ReadyTofire, Enable Controller, and SetSetpoint. Transition to AutoWait. Else moves the ShooterArm
			if (shooterEncoder->Get() < READYTOFIRE + 6
					&& shooterEncoder->Get() > READYTOFIRE - 6) {
				//				pIDControlOutput->PIDWrite(STOPPEDSPEED);
				//				controller->SetPercentTolerance(85);
				//				controller->SetContinuous();
				//				controller->SetSetpoint(shooterEncoder->Get());
				//				controller->Enable();
				cummulativeTime = 0.0;
				pneumaticsControl->compressorDisable();
				autoFireState = AutoWait;
			} else {
				double positionChange = loadRampProfile(timeChange);
				double newSetpoint = controller->GetSetpoint() + positionChange;
				if (newSetpoint >= READYTOFIRE) {
					newSetpoint = READYTOFIRE;
				}
				controller->SetSetpoint(newSetpoint);

			}

			break;
		case AutoWait://A waiting period of 2.5 Seconds. transitions to AutoFire
			cummulativeTime += timeChange;
			if (cummulativeTime >= 2.5) { //waits 2.5 seconds once arm is at readyToFire
				if (canIFire()) {
					autoFireState = AutoFire;
					pIDControlOutput->PIDOverideEnable(AUTOPIDFIRE);
				}
			}
			break;
		case AutoFire://If upperLimit is not Reached, moves the ShooterArm with the controller using shootRampProfile.
			//Once upperLimit and shooterEncoder > Fire, Transition to AutoRetract
			if (isUpperLimit || shooterEncoder->Get() > FIRE) {
				controller->SetSetpoint(FIRE);
				pneumaticsControl->compressorEnable();
				pIDControlOutput->PIDOverideDisable();
				autoFireState = AutoRetract;
			} else {
				double countChange = shootRampProfile(timeChange);
				double newSetpoint = controller->GetSetpoint() + countChange;
				if (newSetpoint >= FIRE) {
					newSetpoint = FIRE;
				}
				controller->Enable();
				controller->SetSetpoint(newSetpoint);
			}
			break;
		case AutoRetract://Lowers the Arm with Controller using downRampProfile. Once the SetPoint is less than HOME, Done firing (The arm should be all the way down)
			double positionChange = downRampProfile(timeChange);
			double newSetpoint = controller->GetSetpoint() + positionChange;
			if (newSetpoint <= HOME) {
				newSetpoint = HOME;
				doneAutoFired = true;
			}
			controller->SetSetpoint(newSetpoint);

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
	//Used only in Autonomous, toggleMotor is true for 2.5 Seconds, then False. 
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

	if (isStartPressed) {//Toggle for Colors
		lightCounter++;
		if (lightCounter > 2) {
			lightCounter = 0;
		}
	}

	if (lightCounter == 0) {//Color Is Red
		LED1->Set(Relay::kOn);
		LED3->Set(Relay::kOff);
	} else if (lightCounter == 1) {//Color is Blue
		LED1->Set(Relay::kOff);
		LED3->Set(Relay::kOn);
	} else if (lightCounter == 2) {//Color is Purple
		LED1->Set(Relay::kOn);
		LED3->Set(Relay::kOn);
	}
}

char*ShooterControl::GetAutoStateString() {
	//Return Char(String) value of current Autonomous State
	switch (autoFireState) {
	case AutoInit:
		return "AutoInit";
	case GoHome:
		return "GoHome";
	case AutoWait:
		return "AutoWait";
	case AutoFire:
		return "AutoFire";
	default:
		return "";
	}
}

bool ShooterControl::doneAutoFire() {//returns true only after done firing // Edit: not need honestly 
	return doneAutoFired;
}

double ShooterControl::downRampProfile(double timeChange) {
	return (timeChange * -RETRACTCPS);
}

double ShooterControl::shootRampProfile(double timeChange) {
	return (timeChange * SHOOTCPS);
}

double ShooterControl::loadRampProfile(double timeChange) {
	return (timeChange * LOADCPS);
}

void ShooterControl::ballGrabber() {
	bool aHeld = xbox->isAHeld();
	double ballGrabberOutput = STOPPEDSPEED;

	switch (fireState) {
	case Home://Home mode is defined by ballGrabber being retracted. With or without the ball
		pneumaticsControl->ballGrabberRetract();
		if (aHeld) {
			ballGrabberOutput = -BALLMOTOR5SPEED;
		} else {
			ballGrabberOutput = STOPPEDSPEED;
		}
		break;
	case Init:
		pneumaticsControl->ballGrabberExtend();
		break;
	case Arming://Extends the ballgrabber. Spins ballMotors on aHeld to pick up the ball
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
	case ReadyToFire://Extends the ballgrabber. Spins ballMotors on aHeld to pick up the ball
		ballGrabberOutput = -BALLMOTOR5SPEED;
		pneumaticsControl->ballGrabberExtend();
		if (pneumaticsControl->ballGrabberIsExtended()) {
			ballGrabberOutput = STOPPEDSPEED;
		}
		break;
	case Passing://Retracts the ballGrabber, and Spins the ballMotors automatically without button pres to release the ball
		pneumaticsControl->ballGrabberRetract();
		if (!pneumaticsControl->ballGrabberIsExtended()) {
			ballGrabberOutput = BALLMOTOR5SPEED;
		}
	default:
		break;
	}

	//Input to BallGrabberMotors 
	BallGrabberMotor5->Set(ballGrabberOutput);
	BallGrabberMotor6->Set(ballGrabberOutput);

}

bool ShooterControl::canIFire() {//A check to see if the ballgrabber is Extended. Returns true or false
	return pneumaticsControl->ballGrabberIsExtended();
}

void ShooterControl::PIDShooter() {//Shooting Using Encoder Count and PIDControl
	bool isRTHeld = xbox->isRightTriggerHeld();
	bool isYPressed = xbox->isYPressed();
	bool isLTHeld = xbox->isLeftTriggerHeld();
	bool isUpperLimit = upperLimit->Get() == 0;
	bool isLowerLimit = lowerLimit->Get() == 0;
	bool isRBHeld = xbox->isRBumperHeld();
	bool isXPressed = xbox->isXPressed();
	bool isBPressed = xbox->isBPressed();

	//for ramping
	double timeChange = (shooterTimer.Get() - previousTime);
	previousTime = shooterTimer.Get();
	int count = shooterEncoder->Get();
	if (maxEncoderValue < count) {
		maxEncoderValue = count;
	}

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

		if (isRBHeld) {
			fireState = ReadyToFire;
			pneumaticsControl->compressorDisable();
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
		} else if ((count > READYTOFIRE - 6) && (count < READYTOFIRE + 6)) {
				if (isYPressed && canIFire()) {//Regular Shoot
					fireState = Firing;
					pIDControlOutput->PIDOverideEnable(MANUALPIDFIRE);
					maxEncoderValue = 0;
				}

				if (isXPressed && canIFire()) {//twoStageFire Shoot. Originally used to be RT Held and X 
					twoStageSetupPosition = 5;
					twoStagePidSetup = -0.08;
					twoStageEndPosition = 250;
					twoStagePidFire = 1.00;
					fireState = StageTwoFire;
					pIDControlOutput->PIDOverideEnable(twoStagePidFire);
					maxEncoderValue = 0;
				}

				if (isBPressed && canIFire()) {//twoStageFire Shoot. Originally used to be RT Held and Y
					twoStagePidFire = 1.00;
					twoStageEndPosition = 225;
					fireState = StageTwoFire;
					pIDControlOutput->PIDOverideEnable(twoStagePidFire);
					maxEncoderValue = 0;
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
	case StageOneFire:
		if (shooterEncoder->Get() <= twoStageSetupPosition) {
			fireState = StageTwoFire;
			pIDControlOutput->PIDOverideEnable(twoStagePidFire);
			maxEncoderValue = 0;
		} else {
			double countChange = shootRampProfile(timeChange);
			double newSetpoint = controller->GetSetpoint() + countChange;
			if (newSetpoint >= twoStageSetupPosition) {
				newSetpoint = twoStageSetupPosition;
			}
			controller->SetSetpoint(newSetpoint);
		}
		break;
	case StageTwoFire:
		if (isUpperLimit || shooterEncoder->Get() >= twoStageEndPosition) {
			controller->SetSetpoint(twoStageEndPosition);
			pIDControlOutput->PIDOverideDisable();
			fireState = Retracting;
		} else {
			double countChange = shootRampProfile(timeChange);
			double newSetpoint = controller->GetSetpoint() + countChange;
			if (newSetpoint >= twoStageEndPosition) {
				newSetpoint = twoStageEndPosition;
			}
			controller->SetSetpoint(newSetpoint);
		}
		break;
	case TrussSetup:
		if (shooterEncoder->Get() <= TRUSSSETUP) {
			fireState = TrussShot;
			pIDControlOutput->PIDOverideEnable(TRUSSPIDFIRE);
			maxEncoderValue = 0;
		} else {
			double countChange = shootRampProfile(timeChange);
			double newSetpoint = controller->GetSetpoint() + countChange;
			if (newSetpoint >= TRUSSSETUP) {
				newSetpoint = TRUSSSETUP;
			}
			controller->SetSetpoint(newSetpoint);
		}
		break;
	case TrussShot://A higher shoot, with lower release point. X must be held.
		if (isUpperLimit || shooterEncoder->Get() >= TRUSS) {
			controller->SetSetpoint(TRUSS);
			pIDControlOutput->PIDOverideDisable();
			fireState = Retracting;
		} else {
			double countChange = shootRampProfile(timeChange);
			double newSetpoint = controller->GetSetpoint() + countChange;
			if (newSetpoint >= TRUSS) {
				newSetpoint = TRUSS;
			}
			controller->SetSetpoint(newSetpoint);
		}
		break;
	case Passing://Starts the ballgrabberMotors in reverse. See BallGrabber Method.
		if (!isRTHeld) {
			fireState = Home;
		}
		break;
	case Firing://While RTHeld, shooter moves using pIDController. Stops when upperLimit or encoder count is reached
		//if (!isRTHeld) {
		if (false) {
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
				//fireState = Fired;
				pIDControlOutput->PIDOverideDisable();
				fireState = Retracting;
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
	case Fired://Moves to Retracting if RTHeld is released and ballgrabber is extended 
		if (!isRTHeld && canIFire()) {
			fireState = Retracting;
		}
		break;
	case Retracting:// Returns to Home. Using downRampProfile. 
		if (this->shooterEncoder->Get() <= HOME + 5) {
			controller->SetSetpoint(HOME);
			pneumaticsControl->compressorEnable();
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

	dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "S: %s C: %i E: %i",
			GetStateString(), count, maxEncoderValue);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "E: %i", maxEncoderValue);

}

char*ShooterControl::GetStateString() {//Returns a String Value of the Different Firing States 
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
	case TrussSetup:
		return "TrussSetup";
	case StageTwoFire:
		return "StageTwoFire";
	case StageOneFire:
		return "StageOneFire";
	default:
		return "";
	}

}

void ShooterControl::ManualShoot() {//Uncomment in Run to use Code. Moves the shooter arm using the right JoyStick
	float rightValue = 0.0;
	rightValue = xbox->getAxisRightY();
	pIDControlOutput->PIDWrite(rightValue / 2.0);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "MotorSpeed %f",
			rightValue);
	dsLCD->UpdateLCD();
}

void ShooterControl::run() {//Run is called in Hohenheim. Methods must be placed here to run on the robot.  
	//ManualShoot();
	ballGrabber();
	PIDShooter();
	toggleColor();
}

