#include "DriveControl.h"
#include "..\SampleLibrary\XboxController.h"
#include "..\SampleLibrary\KinectController.h"
#define NORMAL_MOVE 2153
#define LEFT_MOVE 4006
#define RIGHT_MOVE -2012
#define FORWARD_MOVE 7007
#define BACK_MOVE -11337

//#define SONAR_START -297458
//#define SONAR_RIGHT 34587
//#define SONAR_LEFT 298974
//#define SONAR 2989754

#define PAUSE_STATE 92487
//oscilation states
//#define OSCILATE_START 123456
//#define OSCILATE_FORWARD 666
//#define OSCILATE_BACK 911
//#define PID_START 13345
//#define PID_EXECUTE 14345

#define RIGHT_DIRECTION 900
#define LEFT_DIRECTION 156
#define DISTANCE_THRESHOLD .6
#define DEFAULT_MIN_POWER .44
#define DEFAULT_DEADBAND .16
#define TURN_POWER .3
#define TURNCYCLES 200
#define STRAIGHT_POWER .3
#define STEPS_FORWARD 5
DriveControl::DriveControl() :
	myRobot(1, 2), sonarCenter(3), signalControlVoltage(4), accelFront(6),
			accelBack(7) // as they are declared above.
{

	xbox = XboxController::getInstance();
	DeadbandWidth = DEFAULT_DEADBAND;
	MinPower = DEFAULT_MIN_POWER;
	myRobot.SetExpiration(0.1);
	dsLCD = DriverStationLCD::GetInstance();
	endGameMaxValue = .55;
}
void DriveControl::initializeHybrid() {
	xbox = KinectController::getInstance();
	referenceHigh = 5.0;
	referenceLow = 5.2;
	nextState = NORMAL_MOVE;
	commandState = -1;
	myRobot.SetSafetyEnabled(true);
	count = 0;
	autoCount = STEPS_FORWARD;
}

void DriveControl::initialize() {
	accelFront.SetZero(accelFront.GetAcceleration());
	accelBack.SetZero(accelBack.GetAcceleration());
	xbox = XboxController::getInstance();
	referenceHigh = 5.0;
	referenceLow = 5.2;
	nextState = NORMAL_MOVE;
	commandState = -1;
	myRobot.SetSafetyEnabled(true);
	count = 0;
	autoCount = STEPS_FORWARD;
}
void DriveControl::initializeAutonomous() {
	referenceHigh = 5.0;
	referenceLow = 5.2;
	nextState = NORMAL_MOVE;
	commandState = -1;
	myRobot.SetSafetyEnabled(false);
	count = 0;
	autoCount = STEPS_FORWARD;
}

float DriveControl::scaledOffset(float originalValue, float minValue,
		float maxValue) {

	if (originalValue == 0.0)
		return 0.0;
	if (minValue > maxValue)
		minValue = maxValue;
	if (fabs(originalValue) > 1.0) {
		originalValue = (originalValue > 0) ? 1.0 : -1.0;
	}
	if (originalValue > 0.0) {
		return (maxValue - minValue) * originalValue + minValue;
	} else {
		return (maxValue - minValue) * originalValue - minValue;
	}
}

float DriveControl::scaledOffset(float originalValue, float minValue) {
	return scaledOffset(originalValue, minValue, 1.0);
}
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

bool DriveControl::isDead(float value) {
	return !(value >= DeadbandWidth || value <= -DeadbandWidth);
}

float DriveControl::adjustDeadband(float value) {
	if (isDead(value)) {
		return 0.0;
	} else {
		return scaleValue(value, DeadbandWidth);
	}
}

void DriveControl::MoveForward(float & lefty, float & righty) {
	lefty = STRAIGHT_POWER;
	righty = STRAIGHT_POWER;
	count++;
	if (count == TURNCYCLES) {
		nextState = NORMAL_MOVE;
		count = 0;
	}
}

void DriveControl::act() {
	float lefty = 0.0;
	float righty = 0.0;
	switch (nextState) {
	case NORMAL_MOVE:
		if (!xbox->JogEnabled()) {
			lefty = xbox->getAxisLeftY();
			righty = xbox->getAxisRightY();
			lefty = adjustDeadband(lefty);
			righty = adjustDeadband(righty);
		}

		bool leftJog = xbox->isLeftJogPressed();
		bool rightJog = xbox->isRightJogPressed();
		bool upJog = xbox->isUpJogPressed();
		bool downJog = xbox->isDownJogPressed();
		if (lefty == 0 && righty == 0) {
			if (leftJog) {
				nextState = LEFT_MOVE;
				count = 0;
				lefty = -TURN_POWER;
				righty = TURN_POWER;
			} else if (rightJog) {
				nextState = RIGHT_MOVE;
				count = 0;
				lefty = TURN_POWER;
				righty = -TURN_POWER;
			} else if (upJog) {
				nextState = FORWARD_MOVE;
				count = 0;
				lefty = STRAIGHT_POWER;
				righty = STRAIGHT_POWER;
			} else if (downJog) {
				nextState = BACK_MOVE;
				count = 0;
				lefty = -STRAIGHT_POWER;
				righty = -STRAIGHT_POWER;
			}
		}
		break;
	case LEFT_MOVE:
		lefty = -TURN_POWER;
		righty = TURN_POWER;
		count++;
		if (count == TURNCYCLES) {
			nextState = NORMAL_MOVE;
			count = 0;
		}
		break;

	case RIGHT_MOVE:
		lefty = TURN_POWER;
		righty = -TURN_POWER;
		count++;
		if (count == TURNCYCLES) {
			nextState = NORMAL_MOVE;
			count = 0;
		}
		break;

	case FORWARD_MOVE:
		MoveForward(lefty, righty);
		break;

	case BACK_MOVE:
		lefty = -STRAIGHT_POWER;
		righty = -STRAIGHT_POWER;
		count++;
		if (count == TURNCYCLES) {
			nextState = NORMAL_MOVE;
			count = 0;
		}
		break;
#ifdef Sonar
		case SONAR:
		newX = sonarCenter.getDistanceInInches();
		if (fabs(newX - oldX) < DISTANCE_THRESHOLD) {
			nextState = NORMAL_MOVE;
		} else if (oldX < newX) {
			if (currentDirection == RIGHT_DIRECTION) {
				nextState = SONAR_LEFT;
			} else
			nextState = SONAR_RIGHT;
		} else {
			if (currentDirection == RIGHT_DIRECTION) {
				nextState = SONAR_RIGHT;
			} else
			nextState = SONAR_LEFT;
		}
		oldX = newX;
		break;
		case SONAR_LEFT:
		lefty = -TURN_POWER;
		righty = TURN_POWER;
		currentDirection = LEFT_DIRECTION;
		count++;
		if (count == TURNCYCLES) {
			nextState = PAUSE_STATE;
			commandState = SONAR;
			count = 400;
		}
		break;
		case SONAR_RIGHT:
		lefty = TURN_POWER;
		righty = -TURN_POWER;
		currentDirection = RIGHT_DIRECTION;
		count++;
		if (count == TURNCYCLES) {
			nextState = PAUSE_STATE;
			commandState = SONAR;
			count = 400;
		}
		break;
		case SONAR_START:
		newX = 5869569.0;
		oldX = sonarCenter.getDistanceInInches();
		nextState = SONAR_LEFT;
		currentDirection = LEFT_DIRECTION;
		break;
#endif
	case PAUSE_STATE:
		count--;
		righty = 0;
		lefty = 0;
		if (count == 0) {
			nextState = commandState;
		}
		break;
	default:
		lefty = 0;
		righty = 0;
		//		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Error: Unknown State");
		break;
	}
	myRobot.TankDrive(scaledOffset(lefty, MinPower),
			scaledOffset(righty, MinPower)); // drive with tank style

}
//assuming your already facing it
bool DriveControl::tipBridgeAuto() {
//
//	float lefty = 0.0;
//	float righty = 0.0;
//	char *autoState;
//	switch (nextState) {
//	default:
//		break;
//	case NORMAL_MOVE:
//		if (autoCount > 0) {
//			nextState = FORWARD_MOVE;
//			count = 0;
//			lefty = STRAIGHT_POWER;
//			righty = STRAIGHT_POWER;
//			autoCount--;
//		} else {
//			dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Complete.");
//			return true;
//		}
//		autoState = "NormalMove";
//		break;
//	case FORWARD_MOVE:
//		MoveForward(lefty, righty);
//		autoState = "ForwardMove";
//		break;
//	}
//
//	dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "bridge tip", autoState);
//	myRobot.TankDrive(scaledOffset(lefty, MinPower),
//			scaledOffset(righty, MinPower)); // drive with tank style
	return false;

}

bool DriveControl::RunAuto() {

	float lefty = 0.0;
	float righty = 0.0;
	//char *autoState;
	switch (nextState) {
	default:
		break;
	case NORMAL_MOVE:
		if (autoCount > 0) {
			nextState = FORWARD_MOVE;
			count = 0;
			lefty = STRAIGHT_POWER;
			righty = STRAIGHT_POWER;
			autoCount--;
		} else {
			lefty=0.0;
			righty=0.0;
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Complete.");
			return true;
		}
		//autoState = "NormalMove";
		break;
	case FORWARD_MOVE:
		MoveForward(lefty, righty);
		//autoState = "ForwardMove";
		break;
	}
	//dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "%8s", autoState);
	if(autoCount==0){
		lefty=0.0;
		righty=0.0;
	}
	myRobot.TankDrive(scaledOffset(lefty, MinPower),
			scaledOffset(righty, MinPower)); // drive with tank style
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "left: %6.3f", scaledOffset(lefty, MinPower));
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "right %6.3f", scaledOffset(righty, MinPower));
	return false;
}

float pidDeadZone(float error) {
	return (fabs(error) > .03) ? error : 0.0;
}

void DriveControl::actEndGame() {

	float lefty = 0.0;
	float righty = 0.0;
	if (xbox->isLBumperPressed()) {

		endGameMaxValue -= .02;
		if (endGameMaxValue < 0.0) {
			endGameMaxValue = 0.0;
		}
	}
	if (xbox->isRBumperPressed()) {
		endGameMaxValue += .02;
		if (endGameMaxValue > 1.0) {
			endGameMaxValue = 1.0;
		}
	}

	switch (nextState) {
	case NORMAL_MOVE:
		if (!xbox->JogEnabled()) {
			lefty = xbox->getAxisLeftY();
			righty = xbox->getAxisRightY();
			lefty = adjustDeadband(lefty);
			righty = adjustDeadband(righty);
		}

		bool leftJog = xbox->isLeftJogPressed();
		bool rightJog = xbox->isRightJogPressed();
		bool upJog = xbox->isUpJogPressed();
		bool downJog = xbox->isDownJogPressed();
		if (lefty == 0 && righty == 0) {
			if (leftJog) {
				nextState = LEFT_MOVE;
				count = 0;
				lefty = -TURN_POWER;
				righty = TURN_POWER;
			} else if (rightJog) {
				nextState = RIGHT_MOVE;
				count = 0;
				lefty = TURN_POWER;
				righty = -TURN_POWER;
			} else if (upJog) {
				nextState = FORWARD_MOVE;
				count = 0;
				lefty = STRAIGHT_POWER;
				righty = STRAIGHT_POWER;
			} else if (downJog) {
				nextState = BACK_MOVE;
				count = 0;
				lefty = -STRAIGHT_POWER;
				righty = -STRAIGHT_POWER;
			}
		}
		break;

	case LEFT_MOVE:
		lefty = -TURN_POWER;
		righty = TURN_POWER;
		count++;
		if (count == TURNCYCLES) {
			nextState = NORMAL_MOVE;
			count = 0;
		}
		break;

	case RIGHT_MOVE:
		lefty = TURN_POWER;
		righty = -TURN_POWER;
		count++;
		if (count == TURNCYCLES) {
			nextState = NORMAL_MOVE;
			count = 0;
		}
		break;

	case FORWARD_MOVE:
		MoveForward(lefty, righty);
		break;

	case BACK_MOVE:
		lefty = -STRAIGHT_POWER;
		righty = -STRAIGHT_POWER;
		count++;
		if (count == TURNCYCLES) {
			nextState = NORMAL_MOVE;
			count = 0;
		}
		break;

	case PAUSE_STATE:
		count--;
		righty = 0;
		lefty = 0;
		if (count == 0) {
			nextState = commandState;
		}
		break;
#ifdef Pid
		case OSCILATE_START:
		oscPeriod = 50;
		oscAmp = .2;
		oscCurrentAmp = 0.0;
		oscMaxStep = .005;
		nextState = OSCILATE_BACK;
		lefty = -oscCurrentAmp;
		righty = -oscCurrentAmp;
		break;

		case OSCILATE_FORWARD:
		oscPeriod--;
		oscCurrentAmp += oscMaxStep;
		if (oscCurrentAmp >= oscAmp) {
			oscCurrentAmp = oscAmp;
		}
		if (fabs(oscCurrentAmp) < .5 * oscMaxStep) {
			nextState = PAUSE_STATE;
			count = 200;
			commandState = OSCILATE_FORWARD;
		} else {
			lefty = oscCurrentAmp;
			righty = oscCurrentAmp;
			if (oscPeriod <= 0) {
				oscPeriod = 100;
				nextState = OSCILATE_BACK;
			}
		}
		break;
		case OSCILATE_BACK:
		oscPeriod--;
		oscCurrentAmp -= oscMaxStep;
		if (oscCurrentAmp <= -oscAmp) {
			oscCurrentAmp = -oscAmp;
		}
		if (fabs(oscCurrentAmp) < .5 * oscMaxStep) {
			nextState = PAUSE_STATE;
			count = 200;
			commandState = OSCILATE_BACK;
		} else {
			lefty = oscCurrentAmp;
			righty = oscCurrentAmp;
			if (oscPeriod <= 0) {
				oscPeriod = 100;
				nextState = OSCILATE_FORWARD;
			}
		}
		break;
		case PID_START:
		float error = getPIDError();
		pidDerivative = (error - pidPreviousError) / .2;
		if (fabs(pidDerivative) > fabs(divMax)) {
			divMax = pidDerivative;
		}
		if (error != 0.0) {
			if (fabs(pidDerivative) >= .2) {
				pidOutput = (pidDerivative / fabs(pidDerivative)) * .1;
			} else {
				pidSumOfError += error * .02;
				if (fabs(error) < .05) {
					pidSumOfError = 0.0;
				}
				pidOutput = error * (float) PID_KP + pidSumOfError * PID_KI
				/*+ pidDerivative * PID_KD*/;
				if (pidOutput >= .6) {
					pidOutput = .6;
				}
				if (pidOutput <= -.6) {
					pidOutput = -.6;
				}
			}
			//	lefty = pidOutput;
			//	righty = pidOutput;
			nextState = PID_EXECUTE;
			count = PID_DT;
			pidPreviousError = error;
		} else {
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "flat");
		}
		break;

		case PID_EXECUTE:
		count--;
		//	lefty = pidOutput;
		//	righty = pidOutput;
		if (count <= 1) {
			nextState = PID_START;
		}
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "o%f-s%f-f%f-b%f",
				pidOutput, scaledOffset(pidOutput, MinPower),
				accelFront.GetAcceleration(), accelBack.GetAcceleration());
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "f%f-b%f",
				accelFront.GetAcceleration(), accelBack.GetAcceleration());
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "i%f", pidSumOfError);
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "d%6.3f m%6.3f",
				pidDerivative, divMax);

		break;
#endif
	default:
		lefty = 0;
		righty = 0;
		break;
	}
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "Max Veloc. %6.3f",
			endGameMaxValue);
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "Stick Val %6.3f",
			scaledOffset(righty, .2, endGameMaxValue));
	myRobot.TankDrive(scaledOffset(lefty, .2, endGameMaxValue),
			scaledOffset(righty, .2, endGameMaxValue)); // drive with tank style

}

