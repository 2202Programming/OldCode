#include "DriveControl.h"
#include "XboxController.h"
#include "Math.h"

#define DEFAULT_MIN_POWER .44

//we're not 100% sure about these. I would check but there is no internet. Hey, it works
#define FRONTLEFTMOTOR 3
#define BACKLEFTMOTOR 4
#define FRONTRIGHTMOTOR 1
#define BACKRIGHTMOTOR 2
#define AUTOBACKSPEED .8
DriveControl::DriveControl() :
	myRobot(FRONTLEFTMOTOR, BACKLEFTMOTOR, FRONTRIGHTMOTOR, BACKRIGHTMOTOR) {
	xbox = XboxController::getInstance();
	myRobot.SetExpiration(0.1);
	dsLCD = DriverStationLCD::GetInstance();
	MinPower = DEFAULT_MIN_POWER;
	RightMotorSpeed = 0;
	LeftMotorSpeed = 0;
	precisionDrive = false;
	ArcadeMotorSpeed = 0;
	ArcadeRotateSpeed = 0;
}

void DriveControl::initialize() {
	//xbox = XboxController::getInstance();
	accelTimer.Start();

}
/*
 void DriveControl::initializeAutonomous() {
 myRobot.SetSafetyEnabled(false);
 }

 //min value = .44
 // max value = 1.0
 // original value if at full throttle = 1.0

 float DriveControl::scaledOffset(float originalValue, float minValue,
 float maxValue) {

 if (fabs(originalValue) <= 0.1)
 return 0.0;

 if (minValue > maxValue)
 minValue = maxValue;

 if (fabs(originalValue) > 1.0) {
 originalValue = (originalValue > 0) ? 1.0 : -1.0;
 }

 if (originalValue > 0.0) {
 return (maxValue - minValue) * originalValue + minValue;// (1 - .44)*1 + .44 = 1
 } else {
 return (maxValue - minValue) * originalValue - minValue;
 }
 }
 */
/*
 float DriveControl::scaledOffset(float originalValue, float minValue) {
 return scaledOffset(originalValue, minValue, 1.0);
 }
 */
/*
 float DriveControl::scaleValue(float originalValue, float offset) {
 if (offset != 1 && originalValue != 0) {
 if (originalValue > 0) {
 return (1.0 / (1.0 - offset)) * originalValue - (offset / (1.0
 - offset));
 } else {
 return (1.0 / (1.0 - offset)) * originalValue + (offset / (1.0
 - offset));
 }
 }
 return 0;
 }
 */

//----------------------------------------------------------------------

float DriveControl::accelerateMotor(float stickValue, float MotorValue,
		float loopTimer) {
	bool isBackPressed = xbox->isBackPressed();

	if (isBackPressed) {
		if (precisionDrive) {
			precisionDrive = false;
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "");

		} else {
			precisionDrive = true;
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "MicroCtrl On");
		}
	}
	dsLCD->UpdateLCD();

	if (precisionDrive) {
		stickValue = stickValue * .5;
	}

	// check for dead zone
	if (stickValue < .1 && stickValue > -.1) {
		stickValue = 0;
	}
	// magnitude of acceleration
	bool decel = false;
	if (fabs(stickValue) < fabs(MotorValue)) {
		decel = true;
	}
	float accelMagnitude;
	if (decel) {
		accelMagnitude = .1;
		//accelMagnitude = loopTimer / .9;

	} else {
		accelMagnitude = loopTimer / .8;
	}
	if (accelMagnitude > .25) {
		accelMagnitude = .25;
	}
	// check if we need to decelerate
	if (stickValue < MotorValue) {
		MotorValue -= accelMagnitude;
		// checks if Motor decelerated beyond stickValue then set it back to stick value
		if (MotorValue < stickValue) {
			MotorValue = stickValue;
		}
		// else accelerate
	} else {
		MotorValue += accelMagnitude;
		//checks if Motor accelerated beyond stickValue then set it back to stick value
		if (MotorValue > stickValue) {
			MotorValue = stickValue;
		}

	}

	return MotorValue;

}
//if the back button is pressed, the speed of the motor will be halfed. It is reset if pressed again.
float DriveControl::accelerateTurnMotor(float stickValue, float MotorValue,
		float loopTimer) {
	bool isBackPressed = xbox->isBackPressed();

	if (isBackPressed) {
		if (precisionDrive) {
			precisionDrive = false;

		} else {
			precisionDrive = true;
		}
	}

	if (precisionDrive) {
		stickValue = stickValue * .5;
	}

	// check for dead zone
	if (stickValue < .1 && stickValue > -.1) {
		stickValue = 0;
	}
	// magnitude of acceleration
	bool decel = false;
	if (fabs(stickValue) < fabs(MotorValue)) {
		decel = true;
	}
	float accelMagnitude;
	if (decel) {
		accelMagnitude = .1;
		//accelMagnitude = loopTimer / .9;

	} else {
		accelMagnitude = .1;
	}
	if (accelMagnitude > .25) {
		accelMagnitude = .25;
	}
	// check if we need to decelerate
	if (stickValue < MotorValue) {
		MotorValue -= accelMagnitude;
		// checks if Motor decelerated beyond stickValue then set it back to stick value
		if (MotorValue < stickValue) {
			MotorValue = stickValue;
		}
		// else accelerate
	} else {
		MotorValue += accelMagnitude;
		//checks if Motor accelerated beyond stickValue then set it back to stick value
		if (MotorValue > stickValue) {
			MotorValue = stickValue;
		}

	}

	return MotorValue;

}

void DriveControl::runTank() {
	accelTimer.Stop();
	float timerValue = accelTimer.Get();
	float lefty = 0.0;
	float righty = 0.0;
	lefty = xbox->getAxisLeftY();
	righty = xbox->getAxisRightY(); //this is for tank
	//		myRobot.TankDrive(scaledOffset(lefty, MinPower),
	//				scaledOffset(righty, MinPower)); // drive with tank style

	// match motor speed if stick values are only slightly off to drive straight
	if (fabs(lefty - righty) < .05) {
		lefty = righty;
	}
	LeftMotorSpeed = accelerateMotor(lefty, LeftMotorSpeed, timerValue);
	RightMotorSpeed = accelerateMotor(righty, RightMotorSpeed, timerValue);
	myRobot.TankDrive(-1 * LeftMotorSpeed, -1 * RightMotorSpeed);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "speed: %f",
			RightMotorSpeed);
	dsLCD->UpdateLCD();
	accelTimer.Reset();
	accelTimer.Start();

	//lefty = xbox->getAxisLeftY(); // this is for arcade
	//righty = xbox->getAxisLeftX();
	//myRobot.ArcadeDrive(-scaledOffset(lefty, MinPower),
	//	-scaledOffset(righty, MinPower)); // drive with tank style
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Left: %f",
	//			xbox->getAxisLeftY());
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "right: %f",
	//			xbox->getAxisRightY());
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "scaled left: %f", lefty);
	//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "scaled rt: %f", righty);
	//	dsLCD->UpdateLCD();
}

void DriveControl::runArcade() {
	accelTimer.Stop();
	float timerValue = accelTimer.Get();
	float moveValue = 0.0;
	float rotateValue = 0.0;

	moveValue = xbox->getAxisLeftY();
	rotateValue = xbox->getAxisLeftX();
	ArcadeMotorSpeed = accelerateMotor(moveValue, ArcadeMotorSpeed, timerValue);
	ArcadeRotateSpeed = accelerateTurnMotor(rotateValue, ArcadeRotateSpeed,
			timerValue);

	myRobot.ArcadeDrive(-1 * ArcadeMotorSpeed, ArcadeRotateSpeed);

	accelTimer.Reset();
	accelTimer.Start();

}

// This method does not use acceleration. But it adds friction value to make motor more responsive at lower move values
void DriveControl::runArcadeNoAcceleration() {
	float moveValue = 0.0;
	float rotateValue = 0.0;

	// friction value is added as a constant to motor to make it more responsive to joystick at lower value
	float frictionValue = 0.0;
	float rotateFriction = 0.0;
	float SpeedControl = 1.5;
	moveValue = xbox->getAxisLeftY();
	rotateValue = xbox->getAxisLeftX();

	// if move value is above the dead zone set friction value to .2
	if (moveValue > .1) {
		frictionValue = 0.2;
	} else if (moveValue < -.1) {
		frictionValue = -.2;
	}
	if (rotateValue > .1) {
		rotateFriction = 0.2;
	} else if (rotateValue < -.1) {
		rotateFriction = -.2;
	}

	// if back button is pressed, set the precision drive mode
	bool isBackPressed = xbox->isBackPressed();
	if (isBackPressed) {
		if (precisionDrive) {
			precisionDrive = false;
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "");

		} else {
			precisionDrive = true;
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "MicroCtrl On");
		}
	}
	if (precisionDrive)
		SpeedControl = 2.5;

	myRobot.ArcadeDrive(-1 * ((moveValue + frictionValue) / SpeedControl),
			(rotateValue + rotateFriction) / SpeedControl);

	//dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "speed: %f", moveValue);
//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "rotate: %f", rotateValue);
	dsLCD->UpdateLCD();
}

bool DriveControl::runAuto() {
	myRobot.ArcadeDrive(AUTOBACKSPEED, AUTOBACKSPEED);
	return (true);
}

