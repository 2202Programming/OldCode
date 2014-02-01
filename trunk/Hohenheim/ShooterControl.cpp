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
#define READYTOFIRE 2000
#define FIRE 5000
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
	shooterEncoder = new Encoder(10, 9, true, Encoder::k2X);
	dsLCD = DriverStationLCD::GetInstance();
	pneumaticsControl = PneumaticsControl::getInstance();
	BallGrabberMotor5 = new Victor(BALLGRABBERMOTOR5);
	BallGrabberMotor6 = new Victor(BALLGRABBERMOTOR6);
	UpperShooter = new Talon(UPPERSHOOTER);
	LowerShooter = new Talon(LOWERSHOOTER);
	pIDControlOutput = new PIDControlSubClass(UpperShooter, LowerShooter);
	controller = new PIDController(Kp, Ki, Kd, shooterEncoder, pIDControlOutput);
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
	controller->SetPercentTolerance(85);
	controller->SetContinuous();
	//controller->Enable();
	//controller->SetSetpoint(HOME);
	fireState = Arming;

}

/*If B is Pressed, Depending on the PistonState, Retracts or Fires Pistons
 *If A is Held and Pistons are Fired, BallMotorSpeeds are Set to a Value
 */
void ShooterControl::ballGrabber() {

	bool aHeld = xbox->isAHeld();
	bool bPress = xbox->isBPressed();
	bool yHeld = xbox->isYHeld();
	bool isBallGrabberExtended = pneumaticsControl->ballGrabberIsExtended();

	if (bPress) {
		if (!isBallGrabberExtended) {
			pneumaticsControl->ballGrabberExtend();
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "BG: Ext %i", pneumaticsControl->getCompressorPressure());
		} else {
			pneumaticsControl->ballGrabberRetract();
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3,
					"BG: Ret %i", pneumaticsControl->getCompressorPressure());
			BallGrabberMotor5->Set(STOPPEDSPEED);
			BallGrabberMotor6->Set(STOPPEDSPEED);
		}
	}

	if (aHeld) {
		if (isBallGrabberExtended) {
			if (BallGrabberMotor5->Get() == STOPPEDSPEED) {
				BallGrabberMotor5->Set(BALLMOTOR5SPEED);
				BallGrabberMotor6->Set(BALLMOTOR5SPEED);
			} else {
				BallGrabberMotor5->Set(STOPPEDSPEED);
				BallGrabberMotor6->Set(STOPPEDSPEED);
			}

			//			controller->SetSetpoint(HOME);

		} else {
			BallGrabberMotor5->Set(STOPPEDSPEED);
			BallGrabberMotor6->Set(STOPPEDSPEED);
		}
	}
	//	 else {
	//		loadingBall = false;
	//	}

	if (yHeld) {
		if (isBallGrabberExtended) {
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

	int count = shooterEncoder->Get();
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Count: %i", count);

}

/*
 * Switch and Case
 * Initally Goes to Arming State where The Shooter is Dropped all the way
 * Then the code goes to the Next Case ReadyToFire. ReadyToFire Raises Up until a Limit is Reached. Once the limit is Reached, it waits for X-Pressed
 * Once X-Pressed, Goes to Firing Case. Sets the Shooter To A set Speed Fowards
 */
void ShooterControl::shoot() {
	bool isXPress = xbox->isXPressed();
	bool isUpperLimit = upperLimit->Get();
	bool isLowerLimit = lowerLimit->Get();
	int shooterCount = shooterEncoder->Get();

	switch (fireState) {
	case Arming: {
		//arming
		if (isLowerLimit) {
			UpperShooter->Set(STOPPEDSPEED);
			LowerShooter->Set(STOPPEDSPEED);
			shooterEncoder->Reset();
			fireState = ReadyToFire;
		} else {
			UpperShooter->Set(ARMINGSPEED);
			LowerShooter->Set(ARMINGSPEED);
		}
	}
		break;
	case ReadyToFire: {
		//ready to fire. Lift up until a value is reached
		if (shooterCount >= READYTOFIRELIMIT) {
			if (isXPress) {
				fireState = Firing;
			}
		} else {
			UpperShooter->Set(LOADINGSPEED);
			LowerShooter->Set(LOADINGSPEED);
		}
	}
		break;
	case Firing: {
		//firing
		if (isUpperLimit) {
			UpperShooter->Set(STOPPEDSPEED);
			LowerShooter->Set(STOPPEDSPEED);
			fireState = Arming;
		} else {
			UpperShooter->Set(FIRESPEED);
			LowerShooter->Set(FIRESPEED);
		}
	}
		break;
	}
}

bool ShooterControl::canIFire(){
	return pneumaticsControl->ballGrabberIsExtended();
	//return pneumaticsControl->ballGrabberIsExtended() && pneumaticsControl->getCompressorPressure()>20;
	
}

void ShooterControl::PIDShooter() {
	bool isYPress = xbox->isYPressed();
	bool isUpperLimit = upperLimit->Get();
	bool isLowerLimit = lowerLimit->Get();
	int count = shooterEncoder->Get();

	//	if (!loadingBall) {
	switch (fireState) {
	case Arming: {
		//arming
		//controller->SetSetpoint(HOME);
		if (isLowerLimit) {
			UpperShooter->Set(0.0);
			LowerShooter->Set(0.0);
			shooterEncoder->Reset();
			controller->Enable();
			controller->SetSetpoint(HOME);
			fireState = ReadyToFire;
		}
		else {
			UpperShooter->Set(ARMINGSPEED);
			LowerShooter->Set(ARMINGSPEED);
		}
	}
		break;
	case ReadyToFire: {
		//ready to fire. Lift up until a value is reached
		/*if(controller->Get() < (READYTOFIRE - 50)){ //spin ball grabber motors to help move ball to readyToFire position
		 BallGrabberMotor5->Set(BALLMOTOR5SPEED);
		 BallGrabberMotor6->Set(BALLMOTOR5SPEED);
		 }
		 else{
		 BallGrabberMotor5->Set(STOPPEDSPEED);
		 BallGrabberMotor6->Set(STOPPEDSPEED);
		 }
		 controller->SetSetpoint(READYTOFIRE);
		 */
		
		if (isYPress && canIFire()) {
			// && abs(((int) (controller->Get())) - READYTOFIRE) <= 100
			
			fireState = Firing;
			//immediatly opperate as changing states
			controller->SetSetpoint(FIRE);
		}
	}
		break;
	case Firing: {
		//firing

		if (isYPress) {
//		if (isUpperLimit || isYPress) {
			fireState = Arming;
			controller->Disable();
			
		}
	}
		break;
	}
	//	}
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "S: %i C: %i",
			(int) fireState, count);

}

//Code for shooting if they only want two stage. Without any middle state.
void ShooterControl::twoStageShoot() {
	bool xPressed = xbox->isXPressed();
	bool isLowerLimit = lowerLimit->Get();
	bool isUpperLimit = upperLimit->Get();
	switch (twoStageFire) {
	case Rest: {
		if (isLowerLimit) {
			shooterEncoder->Reset();
			if (xPressed) {
				twoStageFire = Fired;
			}
		} else {
			controller->SetSetpoint(HOME);
		}
	}
		break;
	case Fired: {
		controller->SetSetpoint(PIDFIRE);
		if (isUpperLimit) {
			twoStageFire = Rest;
		}
	}
		break;
	}
}

void ShooterControl::run() {
	ballGrabber();
	//shoot();
	PIDShooter();
	//twoStageShoot();
}

