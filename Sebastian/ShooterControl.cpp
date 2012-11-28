#include "ShooterControl.h"
#include "SebastianShooterMotors.h"
#include "LunacyShooterMotors.h"
#include "LCDShooterMotors.h"
#include "..\SampleLibrary\XboxController.h"
#include "..\SampleLibrary\KinectController.h"
#define STOPPED_STATE 3424958
#define FEEDME_STATE 20394857
#define SHOOT_TOP_STATE 9385493
#define SHOOT_BOT_STATE 3948592
#define FULL_STATE 4923039
#define PUKE_STATE 98493023
#define RECOVERY_STATE 938492
#define MOTOR_OFF 0.0
#define LIFT_UP 1.0
#define LIFT_DOWN -1.0
#define FEED_UP -1.0
#define FEED_DOWN 1.0
#define SWEEPER_IN 1.0
#define SWEEPER_OUT -1.0
#define SHOOTER_ON -0.5
#define SECOND 200
#define AUTO_TIME SECOND*7 

#define LAYUP_STATE 78907
#define LAYUP_SPEED -0.43 //.09
//#define FIVE_FEET_STATE 87628
//#define FIVE_FEET_SPEED -.50
#define KEY1_STATE 5645
#define KEY1_SPEED -0.56
#define HALFCOURT_STATE 22433
#define HALFCOURT_SPEED -0.90
#define MAX_STATE 7776665
#define MAX_SPEED -1.0
#define STEP 0.01
#define SHOOTDIF .1
#define LAYUP_DIFF	.09
#define KEY1_DIFF .11

ShooterControl::ShooterControl() {
	xbox = XboxController::getInstance();
	dsLCDShoot = DriverStationLCD::GetInstance();
	shootSpeedDiff = .09;
	shooterSpeed = LAYUP_SPEED;
	_dsm = new SebastianShooterMotors();
}

void ShooterControl::InitializeAutonomous() {
	currentShooterState = FULL_STATE;
	shooterSpeedState = KEY1_STATE;
	autoCount = 0;
}

void ShooterControl::InitializeOperator() {
	xbox = XboxController::getInstance();
	currentShooterState = FULL_STATE;
	shooterSpeedState = LAYUP_STATE;
}

void ShooterControl::InitializeHybrid() {
	xbox = KinectController::getInstance();
	currentShooterState = FULL_STATE;
	shooterSpeedState = LAYUP_STATE;
}

void ShooterControl::StateChecker() {
	bool wasPuking = (currentShooterState == PUKE_STATE);
	bool stillPuking = xbox->isXHeld();
	if (wasPuking && !stillPuking) {
		currentShooterState = FULL_STATE;
	}

	bool wasShooting = ((currentShooterState == SHOOT_BOT_STATE)
			|| (currentShooterState == SHOOT_TOP_STATE));
	bool leftTrigger = xbox->isLeftTriggerHeld();
	bool rightTrigger = xbox->isRightTriggerHeld();
	bool stillShooting = (leftTrigger || rightTrigger);
	if (wasShooting && !stillShooting) {
		currentShooterState = FULL_STATE;
	}

	if (xbox->isBPressed()) {
		switch (currentShooterState) {
		default:
		case STOPPED_STATE:
		case FULL_STATE:
			currentShooterState = FEEDME_STATE;
			break;
		case FEEDME_STATE:
			currentShooterState = FULL_STATE;
			break;
		}
	}
	if ((currentShooterState == FULL_STATE) || (currentShooterState
			== FEEDME_STATE)) {
		if (rightTrigger) {
			currentShooterState = SHOOT_BOT_STATE;
		} else if (leftTrigger) {
			currentShooterState = SHOOT_TOP_STATE;
		}
	}
	if (xbox->isXHeld()) {
		currentShooterState = PUKE_STATE;
	}

	//	if (xbox->isRBumperPressed()) {
	//
	//		shooterSpeed -= STEP;
	//		if (shooterSpeed <= -1.0) {
	//			shooterSpeed = -1.0;
	//		}
	//	}
	//	if (xbox->isLBumperPressed()) {
	//		shooterSpeed += STEP;
	//		if (shooterSpeed >= 0.0) {
	//			shooterSpeed = 0.0;
	//		}
	//	}
	dsLCDShoot->PrintfLine(DriverStationLCD::kUser_Line4, "Sh.Speed: %6.3f",
			shooterSpeed);
}

void ShooterControl::FeedMeStateControl() {
	_dsm->setFeederMotorSpeed(FEED_UP);
	_dsm->setSweeperMotorSpeed(SWEEPER_IN);
	_dsm->setLiftMotorSpeed(LIFT_UP);
	_dsm->setTopShooterMotorSpeed(LAYUP_SPEED);
	_dsm->setBotShooterMotorSpeed(LAYUP_SPEED);
}

void ShooterControl::FullStateControl() {
	_dsm->setFeederMotorSpeed(MOTOR_OFF);
	_dsm->setSweeperMotorSpeed(SWEEPER_OUT);
	_dsm->setLiftMotorSpeed(MOTOR_OFF);
	_dsm->setTopShooterMotorSpeed(LAYUP_SPEED);
	_dsm->setBotShooterMotorSpeed(LAYUP_SPEED);
}

void ShooterControl::ShootBotStateControl() {
	_dsm->setFeederMotorSpeed(MOTOR_OFF);
	_dsm->setSweeperMotorSpeed(MOTOR_OFF);
	_dsm->setLiftMotorSpeed(LIFT_UP);
	if (shooterSpeedState == LAYUP_STATE) {
		shootSpeedDiff = LAYUP_DIFF;
	}

	if (shooterSpeedState == KEY1_STATE) {
		shootSpeedDiff = KEY1_DIFF;
	}

	float botSpeed = shooterSpeed + shootSpeedDiff;
	if (botSpeed > 0.0)
		botSpeed = 0.0;
	_dsm->setTopShooterMotorSpeed(botSpeed);
	_dsm->setBotShooterMotorSpeed(shooterSpeed);
}

void ShooterControl::ShootTopStateControl() {
	_dsm->setFeederMotorSpeed(FEED_DOWN);
	_dsm->setSweeperMotorSpeed(MOTOR_OFF);
	_dsm->setLiftMotorSpeed(LIFT_UP);
	if (shooterSpeedState == LAYUP_STATE) {
		shootSpeedDiff = LAYUP_DIFF;
	}

	if (shooterSpeedState == KEY1_STATE) {
		shootSpeedDiff = KEY1_DIFF;
	}
	float botSpeed = shooterSpeed + shootSpeedDiff;
	if (botSpeed > 0.0)
		botSpeed = 0.0;
	_dsm->setTopShooterMotorSpeed(botSpeed);
	_dsm->setBotShooterMotorSpeed(shooterSpeed);
}

void ShooterControl::PukeStateControl() {
	_dsm->setFeederMotorSpeed(FEED_DOWN);
	_dsm->setSweeperMotorSpeed(SWEEPER_OUT);
	_dsm->setLiftMotorSpeed(LIFT_DOWN);
	_dsm->setTopShooterMotorSpeed(LAYUP_SPEED);
	_dsm->setBotShooterMotorSpeed(LAYUP_SPEED);
}

void ShooterControl::AutoPukeStateControl() {
	_dsm->setFeederMotorSpeed(FEED_DOWN);
	_dsm->setSweeperMotorSpeed(SWEEPER_OUT);
	_dsm->setLiftMotorSpeed(LIFT_DOWN);
	_dsm->setTopShooterMotorSpeed(MOTOR_OFF);
	_dsm->setBotShooterMotorSpeed(MOTOR_OFF);
}

void ShooterControl::EndGameShoot() {
	_dsm->setFeederMotorSpeed(MOTOR_OFF);
	_dsm->setSweeperMotorSpeed(MOTOR_OFF);
	_dsm->setLiftMotorSpeed(MOTOR_OFF);
	_dsm->setTopShooterMotorSpeed(MOTOR_OFF);
	_dsm->setBotShooterMotorSpeed(MOTOR_OFF);
}

float ShooterControl::GetShooterSpeed() {
	float currentValue = 0.0;
	switch (shooterSpeedState) {
	default:
		break;
	case LAYUP_STATE:
		currentValue = LAYUP_SPEED;
		break;
		//	case FIVE_FEET_STATE:
		//		currentValue = FIVE_FEET_SPEED;
		//		break;
	case KEY1_STATE:
		currentValue = KEY1_SPEED;
		break;
	case HALFCOURT_STATE:
		currentValue = HALFCOURT_SPEED;
		break;
	case MAX_STATE:
		currentValue = MAX_SPEED;
		break;
	}
	//return currentValue;
	return shooterSpeed;
}

void ShooterControl::ShooterStateControl() {
	char *stateString;
	switch (currentShooterState) {
	default:
	case STOPPED_STATE:
	case FEEDME_STATE:
		stateString = "Feedme";
		FeedMeStateControl();
		break;
	case FULL_STATE:
		stateString = "Full";
		FullStateControl();
		break;
	case SHOOT_TOP_STATE:
		stateString = "ShootTop";
		ShootTopStateControl();
		break;
	case SHOOT_BOT_STATE:
		stateString = "ShootBot";
		ShootBotStateControl();
		break;
	case PUKE_STATE:
		stateString = "Puke";
		PukeStateControl();
		break;

	}
	dsLCDShoot->PrintfLine(DriverStationLCD::kUser_Line3, "%8s", stateString);
}

void ShooterControl::ShooterSpeedPrinter() {
	char *shooterStateString;
	switch (shooterSpeedState) {
	default:
		break;
	case LAYUP_STATE:
		shooterStateString = "LAYUP";
		break;
		//	case FIVE_FEET_STATE:
		//		shooterStateString = "FIVE_FEET";
		//		break;
	case KEY1_STATE:
		shooterStateString = "KEY1";
		break;
	case HALFCOURT_STATE:
		shooterStateString = "HALFCOURT";
		break;
	case MAX_STATE:
		shooterStateString = "MAX";
		break;
	}
	//	dsLCDShoot->PrintfLine(DriverStationLCD::kUser_Line3, "%8s",
	//			shooterStateString);
}
#define shootAuto 738947

void ShooterControl::RunAuto() {

#ifdef pukeAuto
	AutoPukeStateControl();
	dsLCDShoot->PrintfLine(DriverStationLCD::kUser_Line3, "autoPuke");
#endif 	

#ifdef shootAuto
	char *autoState;
	shooterSpeedState = KEY1_STATE;
	shooterSpeed = KEY1_SPEED;

	ShootBotStateControl();
	autoState = "A_ShootBot";

	dsLCDShoot->PrintfLine(DriverStationLCD::kUser_Line3, "%8s", autoState);

	dsLCDShoot->PrintfLine(DriverStationLCD::kUser_Line6, "speed%8f",
			shooterSpeed);
#endif
}

void ShooterControl::Run() {
	StateChecker();
	ShooterStateControl();
	//   ShooterSpeedPrinter();

	//dsLCDShoot->PrintfLine(DriverStationLCD::kUser_Line2, "%8f", -shooterSpeed);


	if (xbox->isAPressed()) {
		switch (shooterSpeedState) {
		case KEY1_STATE:
			shooterSpeedState = LAYUP_STATE;
			shooterSpeed = LAYUP_SPEED;
			break;

		case LAYUP_STATE:
			shooterSpeedState = KEY1_STATE;
			shooterSpeed = KEY1_SPEED;
			break;

		default:
			break;
		}
	}
	if (xbox->isRBumperPressed()) {

		shooterSpeed -= STEP;
		if (shooterSpeed <= -1.0) {
			shooterSpeed = -1.0;
		}
	}
	if (xbox->isLBumperPressed()) {
		shooterSpeed += STEP;
		if (shooterSpeed >= 0.0) {
			shooterSpeed = 0.0;
		}
	}
	if (xbox->isYPressed()) {
		shootSpeedDiff += .01;
		if (shooterSpeed >= 0.0) {
			shooterSpeed = 0.0;
		}
	}
	if (xbox->isStartPressed()) {
		shootSpeedDiff -= .01;
		if (shooterSpeed <= -1.0) {
			shooterSpeed = -1.0;
		}
	}
	dsLCDShoot->PrintfLine(DriverStationLCD::kUser_Line5, "shootDiff %6.3f",
			shootSpeedDiff);
}

void ShooterControl::RunEndGame() {
	EndGameShoot();
}
bool ShooterControl::getLED1() {
	return (shooterSpeedState == LAYUP_STATE);
}
bool ShooterControl::getLED2() {
	//return (shooterSpeedState == HALFCOURT_STATE || shooterSpeedState
	//	== MAX_STATE);
	return (shooterSpeedState == LAYUP_STATE);
}
