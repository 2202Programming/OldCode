/* This is the CPP file for Xbox Controller */

#include "DriveControl.h"
#define NORMAL_MOVE 2153
#define LEFT_MOVE 4006
#define RIGHT_MOVE -2012
#define FORWARD_MOVE 7007
#define BACK_MOVE -11337
#define SONAR_START -297458
#define SONAR_RIGHT 34587
#define SONAR_LEFT 298974
#define SONAR 2989754
#define PAUSE_STATE 92487459
#define RIGHT_DIRECTION 900
#define LEFT_DIRECTION 156
#define DISTANCE_THRESHOLD .6
#define DEFAULT_MIN_POWER .44
#define DEFAULT_DEADBAND .16

#define TURN_POWER .3
#define TURNCYCLES 200
#define STRAIGHT_POWER .1

DriveControl::DriveControl() :
	myRobot(1, 2), sonarCenter(3), signalControlVoltage(7) // as they are declared above.
{
	xbox = XboxController::getInstance();
	DeadbandWidth = DEFAULT_DEADBAND;
	MinPower = DEFAULT_MIN_POWER;
	myRobot.SetExpiration(0.1);

}
void DriveControl::initialize() {
	referenceHigh = 5.0;
	referenceLow = 5.2;
	nextState = NORMAL_MOVE;
	commandState = -1;
	myRobot.SetSafetyEnabled(true);
	count = 0;
}
float DriveControl::scaledOffset(float originalValue, float offset) {
	if (originalValue == 0.0)
		return 0.0;

	if (originalValue > 0.0) {
		return (1 - offset) * originalValue + offset;
	} else {
		return (1 - offset) * originalValue - offset;
	}
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
void DriveControl::act() {

	float lefty = 0.0;
	float righty = 0.0;
	switch (nextState) {
	case NORMAL_MOVE:
		lefty = xbox->getAxisLeftY();
		righty = xbox->getAxisRightY();
		//				dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "rawleft: %f",
		//						lefty);
		//				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3,
		//						"rawright: %f", righty);
		lefty = adjustDeadband(lefty);
		righty = adjustDeadband(righty);
		bool bumpLeft = xbox->getLBumper();
		bool bumpRight = xbox->getRBumper();
		bool start = xbox->getStart();
		bool back = xbox->getBack();
		bool buttonX = xbox->getX();
		if (lefty == 0 && righty == 0) {
			if (bumpLeft) {
				nextState = LEFT_MOVE;
				count = 0;
				lefty = -TURN_POWER;
				righty = TURN_POWER;
			} else if (bumpRight) {
				nextState = RIGHT_MOVE;
				count = 0;
				lefty = TURN_POWER;
				righty = -TURN_POWER;
			} else if (start) {
				nextState = FORWARD_MOVE;
				count = 0;
				lefty = STRAIGHT_POWER;
				righty = STRAIGHT_POWER;
			} else if (back) {
				nextState = BACK_MOVE;
				count = 0;
				lefty = -STRAIGHT_POWER;
				righty = -STRAIGHT_POWER;
			}

			else if (buttonX) {
				nextState = SONAR_START;
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
		lefty = STRAIGHT_POWER;
		righty = STRAIGHT_POWER;
		count++;
		if (count == TURNCYCLES) {
			nextState = NORMAL_MOVE;
			count = 0;
		}
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

	case SONAR:
		newX = sonarCenter.getDistanceInInches();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "Fabs: %6.3f",
				fabs(newX - oldX));
		if (fabs(newX - oldX) < DISTANCE_THRESHOLD) {
			nextState = NORMAL_MOVE;
		}

		else if (oldX < newX) {
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
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Error: Unknown State");
		dsLCD->UpdateLCD();
		break;
	}

	/*
	 dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "oleft: %f", lefty);
	 dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "oright: %f",
	 righty);
	 dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "left: %f",
	 scaledOffset(lefty, MinPower));
	 dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "right: %f",
	 scaledOffset(righty, MinPower));*/

	myRobot.TankDrive(scaledOffset(lefty, MinPower),
			scaledOffset(righty, MinPower)); // drive with tank style


	//begining of sonar	
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Dist: %6.3f",
			sonarCenter.getDistanceInInches());
	dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "CVoltage: %6.3f",
			signalControlVoltage.GetVoltage());
	dsLCD->UpdateLCD();

}

