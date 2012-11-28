#include "WPILib.h"
#include <Math.h>
#include "ArcadeDriver.h"
#include "Time.h"
#include "Toggler.h"
#include "Timer.h"
#include "Solenoid.h"
#include "Relay.h"
/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
//public void AutoReleaseTube();
class LogoMotionRobot : public SimpleRobot {
	Timer test;
	RobotDrive myRobot; // robot drive system
	Joystick xbox;
	Joystick stick; // only joystick
	Victor leftMotor; // used to be victor
	Victor rightMotor;//used to be victor
	Victor arm;//changed to victor for coding work. change back to jaguar for competition
	Victor armBoost;//changed to victor for coding work. change back to jaguar for competition.
	//arm is M3 and armboost is M4  
	Jaguar lMotor;
	Jaguar rMotor;
	DigitalInput upperArmLimit;
	DigitalInput lowerArmLimit;
	DigitalInput compressorPressureSwitch;// used to control pneumatics compressor
	Relay compressorPower;
	DigitalOutput hatchOpen;
	DigitalOutput deployMini;
	DigitalOutput openHand;
	Solenoid openHandS;
	Solenoid closeHandS;
	Solenoid openHatchS;
	Solenoid deployMiniS;
	Solenoid retractMiniS;

	Toggler xButtonToggle;
	Toggler hatchOpenToggle;
	Toggler openHandToggle;
	DriverStation* station;
	float lastArmSpeed;
	double lastArmTime;

	static const int BUTTONX = 3;
	static const int BUTTONY = 4;
	static const int BUTTONA = 1;
	static const int BUTTONB = 2;
	static const int TRIGGER = 3; //these are analog inputs for left/right trigger. Left trigger is positive, right trigger is negative
	static const int BUTTONLB = 5; //these are back buttons
	static const int BUTTONRB = 6;
	static const int BUTTONBack = 7;
	static const int BUTTONStart = 8;
	static const int BUTTONL3 = 9; //clicking on the thumbstick left
	static const int BUTTONR3 = 10; //clicking on thumbstick right
	//7 for compressor on
	static const int HANDLATCH = 1; //1=latching hand, and 0 = not latching

public:
	LogoMotionRobot(void) :
				myRobot(1, 2), // these must be initialized in the same order
				stick(1),// as they are declared above.
				xbox(1), leftMotor(3), rightMotor(4), lMotor(2), rMotor(1),
				upperArmLimit(3), lowerArmLimit(2), arm(5), armBoost(6),
				hatchOpen(4), deployMini(5), openHand(6), openHandS(1),
				closeHandS(2), openHatchS(3), deployMiniS(4), retractMiniS(5),
				compressorPressureSwitch(1), compressorPower(4, 7,
						Relay::kBothDirections) {
		myRobot.SetExpiration(0.1);
		lastArmSpeed = 0;
		lastArmTime = 0;
		test.Start();
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void) {
		myRobot.SetSafetyEnabled(false);
		BlindAutonomous();
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void) {
		bool gripperClosed;
		ArcadeDriver Mario(&stick, &lMotor, &rMotor);
		float tolerance=0.2;
		float armSpeed;
		Toggler yButtonToggle;
		//Toggler xButtonToggle;
		//Toggler hatchOpenToggle;
		Toggler deployMiniToggle;
		//Toggler openHandToggle;
		Toggler armReset;
		myRobot.SetSafetyEnabled(true);
		//DriverStation* station;
		station=DriverStation::GetInstance();
		DriverStationLCD* screen;
		screen=DriverStationLCD::GetInstance();
		deployMini.Set(0);
		hatchOpen.Set(0);
		openHand.Set(0);
		//compressorPower.SetDirection(Relay::kBothDirections);
		openHatchS.Set(false);
		deployMiniS.Set(false);
		retractMiniS.Set(false);

		while (IsOperatorControl()) {

			//compressorPower.Set(Relay::kForward);

			//		myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
			//			Wait(0.005);				// wait for a motor update time
			long timerValue=clock();
			bool handButtonState = xbox.GetRawButton(BUTTONY);
			//			bool xButtonState = xbox.GetRawButton(BUTTONX);
			bool pullingRightTrigger = (xbox.GetRawAxis(TRIGGER)<-0.8);
			station->SetDigitalOut(4, pullingRightTrigger);
			float xAxis= xbox.GetRawAxis(1);//xbox.Turning left became - and turning right becomes + (on the left joystick)
			float yAxis= xbox.GetRawAxis(2);//xbox.Turning up became - and turning down becomes + (on the left joystick)
			float qAxis= xbox.GetRawAxis(3);//xbox.pushing the RT-Trigger becomes (-) xbox.pushing the LT-Trigger becomes (+)  (pressing both make it 0) 
			float rAxis= xbox.GetRawAxis(4);//Xbox.Turning the Right Joystick right makes it (+) Turning the Right Joystick left make (-)
			float eAxis= xbox.GetRawAxis(5);//Xbox.Turning the Right Joystick Up makes it (-) Turning the Right Joystick down makes it (+) 
			armSpeed=eAxis*1.0;
			if (fabs(armSpeed)< tolerance)
				armSpeed=0.0;
			//Uncomment once we get limitswitches
			//armReset.DoesNewStateToggle(xbox.GetRawAxis(TRIGGER)>0.8 && lowerArmLimit.Get());
			//if (armReset.GetState())
			//{
			//armSpeed=-1.0;
			//}

			CompressorControl();
			DeployController();
			gripperClosed = GripperController(pullingRightTrigger,
					handButtonState);

			screen->Printf(DriverStationLCD::kUser_Line1, 0,
					"pullingRightTrigger: %i", pullingRightTrigger);
			screen->Printf(DriverStationLCD::kUser_Line2, 1, "upper limit %i ",
					upperArmLimit.Get());
			screen->Printf(DriverStationLCD::kUser_Line3, 2, "lower limit %i ",
					lowerArmLimit.Get());
			//screen->Printf(DriverStationLCD::kUser_Line1, 1, "Left X-axis: %f ",xAxis );

			//screen->Printf(DriverStationLCD::kUser_Line2, 2, "Left Y-axis: %f ",yAxis );
			//screen->Printf(DriverStationLCD::kUser_Line4, 3, "Triggers: %f ",qAxis );
			//screen->Printf(DriverStationLCD::kUser_Line4, 4, "Right X-axis: %f Y-axis: %f ",rAxis,eAxis );
			screen->Printf(DriverStationLCD::kUser_Line4, 3, "Compressor: %i ",
					compressorPressureSwitch.Get() );
			screen->Printf(DriverStationLCD::kUser_Line5, 4, "lastArmSpd: %f",
					lastArmSpeed);
			screen->Printf(DriverStationLCD::kUser_Line6, 5, "lastArmTime: %f",
					lastArmTime);
			//screen->Printf(DriverStationLCD::kUser_Line6, 5, "Right Speed: %f ",rightSpeed );
			//screen->Printf(DriverStationLCD::kUser_Line6, 5, "time: %i ",timerValue );
			//screen->Printf(DriverStationLCD::kUser_Line4, 3, "Stick Angle: %f ",Mario.GetStickAngle());
			screen->UpdateLCD();

			//Mr. Eggers: get limit switches installed, then uncomment these 3 lines.
			// See if you need a ! (logical inverter) in front of the Get calls
			//if ((armSpeed >0 && upperArmLimit.Get()) || (armSpeed <0 && lowerArmLimit.Get()) )	
			//arm.SetSpeed(armSpeed); //unaccelerated arm motion
			ArmAcceleration(armSpeed, gripperClosed);//accelerated arm motion 

			//else
			//arm.SetSpeed(0.0);
			Mario.Drive2();

		}
	}

	// Automated sequence for raising arm and placing into correct position for teleop.
	// raises arm and turns on solenoid for openhand and then turns off after 2 seconds
	void LogoMotionRobot::BlindAutonomous() {
		arm.SetSpeed(-0.6);
		armBoost.SetSpeed(-0.6);
		Wait(1);
		arm.SetSpeed(0.6);
		armBoost.SetSpeed(0.6);
		Wait(0.5);
		arm.SetSpeed(0);
		armBoost.SetSpeed(0);
		openHandS.Set(true);
		Wait(2.0);
		openHandS.Set(false);

	}

	// Turns compressor on and off
	void LogoMotionRobot::CompressorControl() {
		if (compressorPressureSwitch.Get()) {
			compressorPower.Set(Relay::kOff);
		} else {
			compressorPower.Set(Relay::kForward);
		}
	}

	// Opens hatch or deploys Mini-bot 
	void LogoMotionRobot::DeployController() {
		if (hatchOpenToggle.DoesNewStateToggleDB(xbox.GetRawButton(BUTTONLB)
				&& xbox.GetRawButton(BUTTONRB))) {
			//Pressing both Left and Right buttons
			if (hatchOpenToggle.GetState()) {
				station->SetDigitalOut(7, 1);
				hatchOpen.Set(1);
				openHatchS.Set(true);

			} else {
				openHatchS.Set(false);
				deployMini.Set(1);
				station->SetDigitalOut(8, 1);
				deployMiniS.Set(true);
				Wait(1.0);
				deployMiniS.Set(false);
				Wait(5.0);
				station->SetDigitalOut(8, 1);
				retractMiniS.Set(true);
				Wait(2.0);
				retractMiniS.Set(false);
			}

		}
	}

	// Opens and closes gripper hand by activating solenoids
	bool LogoMotionRobot::GripperController(bool activate, bool handButtonState) {
		bool newHandState = false;

		//This is latching hand (HANDLATCH == 1)
		if (xButtonToggle.DoesNewStateToggleDBI(activate)) {
			openHandS.Set(xButtonToggle.GetState());
			station->SetDigitalOut(1, xButtonToggle.GetState());
			newHandState = !xButtonToggle.GetState();
			closeHandS.Set(!xButtonToggle.GetState());
			station->SetDigitalOut(2, !xButtonToggle.GetState());
			station->SetDigitalOut(6, xButtonToggle.GetState());
		}

		if (openHandToggle.DoesNewStateToggleDBI(handButtonState)) {
			//				openHand.Set(openHandToggle.GetState());
			//openHandS.Set(openHandToggle.GetState());
			station->SetDigitalOut(1, openHandToggle.GetState());
			//closeHandS.Set(!openHandToggle.GetState());
			station->SetDigitalOut(2, !openHandToggle.GetState());
		}
		//		}

		if (openHandToggle.DoesNewStateToggle(false)) //right trigger
		{
			openHand.Set(1);
		}
		return newHandState;
	}

	// Autonomous mode sequence with no visual input.
	// Raises tube to height of peg, moves forward, then releases tube 
	void LogoMotionRobot::AutoReleaseTube() {
		arm.SetSpeed(0.5); //Raise arm
		Wait(2.0);//experiment to find the right value (motor speeds too)
		arm.SetSpeed(0.0); //Arm should be at top
		//		lMotor.SetSpeed(-0.5);	//Move forward
		//	rMotor.SetSpeed(0.5);		
		Wait(2.0);//experiment to find the right value
		//	lMotor.SetSpeed(0);	//Stop
		//rMotor.SetSpeed(0);
		AutoReleaseTube();

		//design notes

		//grip hand closed---------------------
		//station->SetDigitalOut(1,true);
		//station->SetDigitalOut(2,false);
		//Wait(?.?)
		//raise arm to position----------------
		//arm.SetSpeed(0.5);
		//Wait(?.?);
		//arm.SetSpeed(0.0);
		//Wait(?.?);
		//robot foward ()feet------------------
		//lMotor.SetSpeed(0.5);
		//rMotor.SetSpeed(0.5);		
		//Wait(?.?);
		//lMotor.SetSpeed(0);
		//rMotor.SetSpeed(0);
		//Wait(?.?);
		//lower arm----------------------------
		//do we need to do this??
		//grip hand open-----------------------
		//station->SetDigitalOut(1,false);
		//station->SetDigitalOut(2,true);
		//robot backed up----------------------
		//lMotor.SetSpeed(-0.5);
		//rMotor.SetSpeed(-0.5);		
		//Wait(?.?);
		//lMotor.SetSpeed(0);
		//rMotor.SetSpeed(0);		
	}
	void LogoMotionRobot::SolenoidSafetyTest() {

	}

	void LogoMotionRobot::ArmAcceleration(float targetSpeed, bool gripperClosed) {
		double timeNow = test.Get();
#ifdef JUNK
		if(targetSpeed>0)

		targetSpeed *= 0.35;

		else
		targetSpeed *= 0.65;
		if(lastArmTime+0.1<timeNow)
		{
			float diffSpeed = targetSpeed-lastArmSpeed;
			if(fabs(diffSpeed)>0.05)
			{
				if(diffSpeed>0)
				diffSpeed = 0.05;
				else
				diffSpeed = -0.05;
			}
			lastArmSpeed += diffSpeed;
#endif
		{
			lastArmSpeed = targetSpeed;
			if (lastArmSpeed==0) {
				arm.SetSpeed(0);
				armBoost.SetSpeed(0);
			} else {

				if (lastArmSpeed>0) {
					//arm is moving down
					if (gripperClosed) {
						lastArmSpeed = targetSpeed *0.8;
					} else {
						lastArmSpeed = targetSpeed *0.6;
					}

					if (lowerArmLimit.Get() || true) {
						//okay to move the arm
						//take || true off during competition
						arm.SetSpeed(lastArmSpeed);
						armBoost.SetSpeed(lastArmSpeed);
					} else {
						arm.SetSpeed(0.0);
						armBoost.SetSpeed(0);
					}
				} else {
					//arm is moving up

					if (gripperClosed) {
						lastArmSpeed = targetSpeed;
					} else {
						lastArmSpeed = targetSpeed *0.8;
					}
					if (upperArmLimit.Get() || true) {
						//take || true off during competition
						//okay to move the arm
						arm.SetSpeed(lastArmSpeed);
						armBoost.SetSpeed(lastArmSpeed);
					} else {
						arm.SetSpeed(0.0);
						armBoost.SetSpeed(0);
					}
				}

			}

			lastArmTime = timeNow;

		}
	}
};

START_ROBOT_CLASS(LogoMotionRobot)
;

