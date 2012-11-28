#include "WPILib.h"

#include "Toggler.h"
#include "Nivision.h"
//#include "Servo.h"

#define CONVEYOR_UP_SPEED -1.0
#define CONVEYOR_DOWN_SPEED 1.0
#define SWEEPER_IN_SPEED -1.0
#define SWEEPER_OUT_SPEED 1.0
#define MOTOR_OFF 0.0
#define HEADRIGHT_BUTTON 1
#define HEADLEFT_BUTTON 2  
#define RIGHTLEGOUT_BUTTON 3
#define LEFTLEGOUT_BUTTON 4
#define RIGHTLEGFOWARD_BUTTON 5
#define RIGHTLEGBACKWARD_BUTTON 6
#define LEFTLEGFOWARD_BUTTON 7
#define LEFTLEGBACKWARD_BUTTON 8

/**
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class RobotDemo: public SimpleRobot {
	RobotDrive myRobot; // robot drive system
	Joystick lstick; // only joystick
	Joystick rstick;
	Relay light1;
	Relay light2;
	DigitalInput limitSwitch;
	DigitalInput limitSwitch2;
	DriverStation* ds;
	Timer testTimer;
	Jaguar turretRotation;
	Jaguar conveyor;
	Jaguar sweeper;
	Jaguar shooter;
	Toggler teachMode;
	float openAngle;
	float closedAngle;
	KinectStick kinectLeftArm; //The Left arm should be constructed as stick 1
	KinectStick kinectRightArm; //The Right arm should be constructed as stick 2
	DriverStationLCD *dsLCD;

public:
	/*------------------------------------------------------------
	 Method Name: RobotDemo
	 Description: Constructor 
	 Parameters: N/A
	 Returns: New instance of RobotDemo.
	 ------------------------------------------------------------*/
	RobotDemo(void) :
				myRobot(1, 2),
				lstick(1), // Driver Left
				rstick(1), // Driver Right
				kinectLeftArm(1), // as they are declared above.
				kinectRightArm(2), light1(1, Relay::kForwardOnly),

				light2(2, Relay::kForwardOnly), limitSwitch(1),
				limitSwitch2(2), turretRotation(3), conveyor(5), sweeper(6),
				shooter(4) {
		ds = DriverStation::GetInstance();
		/*	myRobot.SetMaxMotorSpeed(1.0);
		 myRobot.SetMotorSpeedScalar(1.0);
		 myRobot.SetMaxAccel(0.1);
		 myRobot.SetAccelInterval(0.1);
		 */
		GetWatchdog().SetExpiration(100);
		openAngle = 2.0;
		closedAngle = 50.0;
		rstick.SetAxisChannel(Joystick::kXAxis, 4);
		rstick.SetAxisChannel(Joystick::kYAxis, 5);
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "KinectLunacy");
		dsLCD->UpdateLCD();
	}

	/*------------------------------------------------------------
	 Method Name: Autonomous
	 Description: Handles operations for automatic control mode.
	 Parameters: N/A
	 Returns: N/A
	 ------------------------------------------------------------*/
	void Autonomous(void) {
		Toggler sweeperConveyorOff;
		Toggler conveyorDown;
		Toggler sweeperIn;
		Toggler conveyorUp;
		Toggler sweeperOut;
		Toggler conveyorInc;
		Toggler conveyorDec;
		float conveyorSpeed = CONVEYOR_DOWN_SPEED * 0.31;
		float sweeperSpeed = SWEEPER_OUT_SPEED;
		bool conveyorOn = false;

		dsLCD->Clear();
		dsLCD->UpdateLCD();

		GetWatchdog().SetEnabled(true);
		while (IsAutonomous()) {
			GetWatchdog().Feed();
			conveyorOn =
					kinectRightArm.GetRawButton(HEADRIGHT_BUTTON);//Left bumper
			bool conveyerUp =
					kinectRightArm.GetRawButton(HEADRIGHT_BUTTON);//Left bumper
			bool conveyerDown = kinectRightArm.GetRawButton(
					HEADLEFT_BUTTON);//Right bumper

			//			bool conveyerFaster = rstick.GetRawButton(9);//Button 
			//			bool conveyerSlower = rstick.GetRawButton(10);//Button 

			bool sweepIn = kinectRightArm.GetRawButton(LEFTLEGFOWARD_BUTTON);//Button A
			bool sweepOut = kinectRightArm.GetRawButton(LEFTLEGBACKWARD_BUTTON);//Button B

			//			bool sweepFaster = kinectRightArm.GetRawButton(3);//Button X
			//			bool sweepSlower = kinectRightArm.GetRawButton(4);//Button y 

			//			bool sweepToggle = kinectRightArm.GetRawButton(8);//Button start 

			float shooterDirection = kinectRightArm.GetX();// right stick x axis 
			float shooterSpeed = 1.0f * kinectRightArm.GetY();// right stick y axis 

			//			float trigger = kinectRightArm.GetRawAxis(3);// left stick y axis 


			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Xbox Lunacy");
			dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
					"Conveyer up:%i down:%i", conveyerUp, conveyerDown);
			//			dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
			//					"Sw In:%i Out:%i t:%i", sweepIn, sweepOut, sweepToggle);
			//			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
			//					"Swe Fast:%i Slow:%i", sweepFaster, sweepSlower);
			dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,
					"Shoot D:%.1f V:%.1f", shooterDirection, shooterSpeed);
			//			dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Shoot:%.1f",
			//					trigger);
			dsLCD->UpdateLCD();

//			/*
//			 * Turns the sweeper and conveyor off together.
//			 * BUTTON: Start 
//			 */
//
////			if (sweeperConveyorOff.DoesNewStateToggle(sweepToggle)) {
////				if (conveyorOn) {
////					conveyor.Set(MOTOR_OFF);
////					sweeper.Set(MOTOR_OFF);
////					conveyorDown.SetState(false);
////					sweeperIn.SetState(false);
////					conveyorUp.SetState(false);
////					sweeperOut.SetState(false);
////					conveyorOn = false;
////				} else {
////
////					conveyorOn = true;
////				}
////			}
//
//			/*
//			 * Increases the conveyor speed by a set amount of 0.25 with every press
//			 * of left bumper.
//			 */
//			if (conveyorInc.DoesNewStateToggle(conveyerUp)) {
//				if (conveyorSpeed <= 1.0)
//					conveyorSpeed += 0.5;//0.125;
//
//				//				if (conveyorUp.GetState())
//				//					conveyor.Set(-1.0 * conveyorSpeed);
//				//				else if (conveyorDown.GetState())
//				//					conveyor.Set(conveyorSpeed);
//
//
//				conveyorOn = true;
//			}
//
//			/*
//			 * Decreases the conveyor speed by a set amount of 0.25 with every press
//			 * of right bumper.
//			 */
//			if (conveyorDec.DoesNewStateToggle(conveyerDown)) {
//				if (conveyorSpeed >= -1.0)
//					conveyorSpeed -= 0.5;//0.125;
//
//				//				if (conveyorUp.GetState())
//				//					conveyor.Set(-1.0 * conveyorSpeed);
//				//				else if (conveyorDown.GetState())
//				//					conveyor.Set(conveyorSpeed);
//
//
//				conveyorOn = true;
//
//			}
//
//			//Turret Direction
//			if (!limitSwitch.Get() && shooterDirection > 0.0)
//				turretRotation.Set(shooterDirection * 0.6);
//			else if (!limitSwitch2.Get() && shooterDirection < 0.0)
//				turretRotation.Set(shooterDirection * 0.6);
//			else
//				turretRotation.Set(0.0);
//
//			/*
//			 * If trigger2 is pressed, the shooter will turn at the direction of the stick
//			 * and the conveyor will start.
//			 * Otherwise, turns the motor off and conveyor on if it was previously on.
//			 */
////			if (trigger) {
////				shooter.Set(0.7 - shooterSpeed * 0.3);
////			} else
////				shooter.Set(MOTOR_OFF);
//
//			//Motor Control
			if (conveyorOn)
				conveyor.Set(1);
			else
				conveyor.Set(MOTOR_OFF);

			//myRobot.ArcadeDrive(lstick);
			myRobot.ArcadeDrive(kinectLeftArm.GetY() * -1.0,
					kinectLeftArm.GetX() * -1.0);
		}
	}

	void OperatorControl(void) {
		Toggler sweeperConveyorOff;
		Toggler conveyorDown;
		Toggler sweeperIn;
		Toggler conveyorUp;
		Toggler sweeperOut;
		Toggler conveyorInc;
		Toggler conveyorDec;
		float conveyorSpeed = CONVEYOR_DOWN_SPEED * 0.31;
		float sweeperSpeed = SWEEPER_OUT_SPEED;
		bool conveyorOn = false;

		dsLCD->Clear();
		dsLCD->UpdateLCD();

		GetWatchdog().SetEnabled(true);
		while (IsOperatorControl()) {
			GetWatchdog().Feed();

			bool conveyerUp = rstick.GetRawButton(5);//Left bumper
			bool conveyerDown = rstick.GetRawButton(6);//Right bumper

			//			bool conveyerFaster = rstick.GetRawButton(9);//Button 
			//			bool conveyerSlower = rstick.GetRawButton(10);//Button 

			bool sweepIn = rstick.GetRawButton(1);//Button A
			bool sweepOut = rstick.GetRawButton(2);//Button B

			bool sweepFaster = rstick.GetRawButton(3);//Button X
			bool sweepSlower = rstick.GetRawButton(4);//Button y 

			bool sweepToggle = rstick.GetRawButton(8);//Button start 

			float shooterDirection = rstick.GetRawAxis(4);// right stick x axis 
			float shooterSpeed = 1.0f * rstick.GetRawAxis(5);// right stick y axis 

			float trigger = rstick.GetRawAxis(3);// left stick y axis 


			dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Xbox Lunacy");
			dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
					"Conveyer up:%i down:%i", conveyerUp, conveyerDown);
			dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"Sw In:%i Out:%i t:%i", sweepIn, sweepOut, sweepToggle);
			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
					"Swe Fast:%i Slow:%i", sweepFaster, sweepSlower);
			dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,
					"Shoot D:%.1f V:%.1f", shooterDirection, shooterSpeed);
			dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Shoot:%.1f",
					trigger);
			dsLCD->UpdateLCD();

			/*
			 * Turns the sweeper and conveyor off together.
			 * BUTTON: Start 
			 */

			if (sweeperConveyorOff.DoesNewStateToggle(sweepToggle)) {
				if (conveyorOn) {
					conveyor.Set(MOTOR_OFF);
					sweeper.Set(MOTOR_OFF);
					conveyorDown.SetState(false);
					sweeperIn.SetState(false);
					conveyorUp.SetState(false);
					sweeperOut.SetState(false);
					conveyorOn = false;
				} else {

					conveyorOn = true;
				}
			}

			/*
			 * Increases the conveyor speed by a set amount of 0.25 with every press
			 * of left bumper.
			 */
			if (conveyorInc.DoesNewStateToggle(conveyerUp)) {
				if (conveyorSpeed <= 1.0)
					conveyorSpeed += 0.5;//0.125;

				//				if (conveyorUp.GetState())
				//					conveyor.Set(-1.0 * conveyorSpeed);
				//				else if (conveyorDown.GetState())
				//					conveyor.Set(conveyorSpeed);


				conveyorOn = true;
			}

			/*
			 * Decreases the conveyor speed by a set amount of 0.25 with every press
			 * of right bumper.
			 */
			if (conveyorDec.DoesNewStateToggle(conveyerDown)) {
				if (conveyorSpeed >= -1.0)
					conveyorSpeed -= 0.5;//0.125;

				//				if (conveyorUp.GetState())
				//					conveyor.Set(-1.0 * conveyorSpeed);
				//				else if (conveyorDown.GetState())
				//					conveyor.Set(conveyorSpeed);


				conveyorOn = true;

			}

			//Turret Direction
			if (!limitSwitch.Get() && shooterDirection > 0.0)
				turretRotation.Set(shooterDirection * 0.6);
			else if (!limitSwitch2.Get() && shooterDirection < 0.0)
				turretRotation.Set(shooterDirection * 0.6);
			else
				turretRotation.Set(0.0);

			/*
			 * If trigger2 is pressed, the shooter will turn at the direction of the stick
			 * and the conveyor will start.
			 * Otherwise, turns the motor off and conveyor on if it was previously on.
			 */
			if (trigger) {
				shooter.Set(0.7 - shooterSpeed * 0.3);
			} else
				shooter.Set(MOTOR_OFF);

			//Motor Control
			if (conveyorOn)
				conveyor.Set(conveyorSpeed);
			else
				conveyor.Set(MOTOR_OFF);

			//myRobot.ArcadeDrive(lstick);
			myRobot.ArcadeDrive(lstick.GetY() * -1.0, lstick.GetX() * -1.0);
		}
	}

	/*------------------------------------------------------------
	 Method Name: OperatorControl
	 Description: Handles operations for manual control mode.
	 Parameters: N/A
	 Returns: N/A
	 ------------------------------------------------------------*/
#ifdef JUNK
	void OldOperatorControl(void) {
		Toggler driveType(true);
		Toggler greenlight;
		Toggler gearUp;
		Toggler gearDown;
		Toggler conveyorUp;
		Toggler sweeperIn;
		Toggler conveyorDown;
		Toggler sweeperOut;
		Toggler moveUp;
		Toggler moveDown;
		Toggler rateUp;
		Toggler rateDown;
		Toggler saveOpen;
		Toggler saveClose;
		Toggler setToMax;
		Toggler setToMin;
		Toggler moveHopperDoor;
		Toggler sweeperConveyorOff;
		Toggler sweeperInc;
		Toggler sweeperDec;
		Toggler conveyorInc;
		Toggler conveyorDec;
		float conveyorSpeed= CONVEYOR_DOWN_SPEED * 0.31;
		float sweeperSpeed= SWEEPER_OUT_SPEED;
		bool wasShooting = false;

		//	myRobot.SetKillAccel(false);
		GetWatchdog().SetEnabled(true);
		myRobot.Drive(0.0, 0.0);

		while (IsOperatorControl()) {
			GetWatchdog().Feed();

			//			bool trigger2 = stick.GetRawButton(1);

			//			float speed = -stick3.GetX();

			/*
			 * Moves the turret left or right under operator driving until it reaches either
			 * limit switches.
			 */
			if (!limitSwitch.Get() && speed > 0.0)
			turretRotation.Set(speed*0.6);
			else if (!limitSwitch2.Get() && speed < 0.0)
			turretRotation.Set(speed*0.6);
			else
			turretRotation.Set(0.0);

			/*
			 * Overrides the velocity acceleration while button is held.
			 * The normal acceleration is implemented.
			 */
			//		myRobot.SetKillAccel(stick.GetRawButton(3));

			//			driveType.DoesNewStateToggle(stick2.GetRawButton(8));

			/*
			 * Determines the drive through the triggers and the driveType.
			 * If driveType is false, it is arcade drive.
			 * If driveType is true, it is tank drive.
			 * Stick 3 is unchanged.
			 */
			//	//		if (trigger && !driveType.GetState())
			//				myRobot.ArcadeDrive(stick, false);
			////			else if ((trigger && trigger3 && driveType.GetState()) 
			//					|| (stick.GetRawButton(3) && stick3.GetRawButton(3) 
			//							&& !driveType.GetState()))
			//				myRobot.TankDrive(stick, stick2);
			//			else
			//				myRobot.Drive(0, 0);

			/*
			 * Increases the Max Acceleration.
			 */
			//			if (gearUp.DoesNewStateToggle(stick.GetRawButton(5))) {
			//				float newAccel = myRobot.GetMaxAccel() + 0.05;
			//				if (newAccel > 1.0)
			//					newAccel = 1.0;
			//
			//				myRobot.SetMaxAccel(newAccel);
			//			}

			/*
			 * Decreases the Max Acceleration.
			 */
			//			if (gearDown.DoesNewStateToggle(stick.GetRawButton(4))) {
			//				float newAccel = myRobot.GetMaxAccel() - 0.05;
			//				if (newAccel < 0.05)
			//					newAccel = 0.05;
			//
			//				myRobot.SetMaxAccel(newAccel);
			//			}

			/*
			 * If trigger2 is pressed, the shooter will turn at the direction of the stick
			 * and the conveyor will start.
			 * Otherwise, turns the motor off and conveyor on if it was previously on.
			 */
			if (trigger2) {
				shooter.Set(0.7 - stick3.GetY() * 0.3);
				conveyor.Set(conveyorSpeed * -1.0);
				wasShooting = true;
			} else {
				if (wasShooting) {
					shooter.Set(MOTOR_OFF);
					if (conveyorDown.GetState())
					conveyor.Set(conveyorSpeed);
					else if (!conveyorUp.GetState())
					conveyor.Set(0.0);
				} else {
					//					if (conveyorUp.DoesNewStateToggle(stick3.GetRawButton(3)
					//							|| stick2.GetRawButton(3))) {
					//turn conveyor on in "up" direction.
					//or if already moving upwards, then stop.
					conveyorDown.SetState(false);
					if (conveyorUp.GetState())
					conveyor.Set(conveyorSpeed * -1.0);
					else
					conveyor.Set(MOTOR_OFF);
				} else if (conveyorDown.DoesNewStateToggle(stick3.GetRawButton(2)
								|| stick2.GetRawButton(2))) {
					//turn conveyor on in "down" direction.
					//or if already moving down, then stop.
					conveyorUp.SetState(false);
					if (conveyorDown.GetState())
					conveyor.Set(conveyorSpeed);
					else
					conveyor.Set(MOTOR_OFF);
				}
			}
			wasShooting = false;
		}

		/*
		 * Turns the sweeper and conveyor off together.
		 * BUTTON: 2-9 or 3-9
		 */

		if (sweeperConveyorOff.DoesNewStateToggle(stick3.GetRawButton(9)
						|| stick2.GetRawButton(9))) {
			conveyor.Set(MOTOR_OFF);
			sweeper.Set(MOTOR_OFF);
			conveyorDown.SetState(false);
			sweeperIn.SetState(false);
			conveyorUp.SetState(false);
			sweeperOut.SetState(false);
		}

		/*
		 * Increases the conveyor speed by a set amount of 0.25 with every press
		 * of button 6.
		 */
		if (conveyorInc.DoesNewStateToggle(stick3.GetRawButton(6)
						|| stick2.GetRawButton(6))) {
			if (conveyorSpeed <= 0.6)
			conveyorSpeed += 0.03;//0.125;

			if (conveyorUp.GetState())
			conveyor.Set(-1.0 * conveyorSpeed);
			else if (conveyorDown.GetState())
			conveyor.Set(conveyorSpeed);
		}

		/*
		 * Decreases the conveyor speed by a set amount of 0.25 with every press
		 * of button 7.
		 */
		if (conveyorDec.DoesNewStateToggle(stick3.GetRawButton(7)
						|| stick2.GetRawButton(7))) {
			if (conveyorSpeed >= 0.1)
			conveyorSpeed -= 0.03; //0.125;

			if (conveyorUp.GetState())
			conveyor.Set(-1.0 * conveyorSpeed);
			else if (conveyorDown.GetState())
			conveyor.Set(conveyorSpeed);
		}

		/**
		 * Turns the sweeper in, or turn it off if pressed again.
		 * Button: 2-4 or 3-4
		 */
		if (sweeperIn.DoesNewStateToggle(stick3.GetRawButton(4)
						|| stick2.GetRawButton(4))) {
			sweeperOut.SetState(false);
			if (sweeperIn.GetState())
			sweeper.Set(-1.0 * sweeperSpeed);
			else
			sweeper.Set(MOTOR_OFF);
		}

		/**
		 * Turns the sweeper in the OUT direction and pushing the balls
		 *  out, or turns it off.
		 * Button: 2-5 or 3-5
		 */
		else if (sweeperOut.DoesNewStateToggle(stick3.GetRawButton(5)
						|| stick2.GetRawButton(5))) {
			//turn sweeper on to push balls out.
			//or if already pushing balls out, then stop.
			sweeperIn.SetState(false);
			if (sweeperOut.GetState())
			sweeper.Set(sweeperSpeed);
			else
			sweeper.Set(MOTOR_OFF);
		}

		/*
		 * Increases the sweeper speed by a set amount of 0.25 with every press
		 * of button 11.
		 */
		if (sweeperInc.DoesNewStateToggle(stick3.GetRawButton(11)
						|| stick2.GetRawButton(11))) {
			if (sweeperSpeed <= 0.75) {
				sweeperSpeed += 0.25;
			}
			if (sweeperIn.GetState())
			sweeper.Set(sweeperSpeed);
			else if (sweeperOut.GetState())
			sweeper.Set(-1.0 * sweeperSpeed);
		}

		/*
		 * Decreases the sweeper speed by a set amount of 0.25 with every press
		 * of button 10.
		 */
		if (sweeperDec.DoesNewStateToggle(stick3.GetRawButton(10)
						|| stick2.GetRawButton(10))) {
			if (sweeperSpeed >= 0.25) {
				sweeperSpeed -= 0.25;
			}
			if (sweeperIn.GetState())
			sweeper.Set(sweeperSpeed);
			else if (sweeperOut.GetState())
			sweeper.Set(-1.0 * sweeperSpeed);
		}
	}

}
#endif
};
START_ROBOT_CLASS(RobotDemo)
;

