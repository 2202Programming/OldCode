#include "WPILib.h"
#define HEADRIGHT_BUTTON 1
#define HEADLEFT_BUTTON 2  
#define RIGHTLEGOUT_BUTTON 3
#define LEFTLEGOUT_BUTTON 4
#define RIGHTLEGFOWARD_BUTTON 5
#define RIGHTLEGBACKWARD_BUTTON 6
#define LEFTLEGFOWARD_BUTTON 7
#define LEFTLEGBACKWARD_BUTTON 8


/**
 * This code demonstrates the use of the KinectStick
 * class to drive your robot during the autonomous mode
 * of the match, making it a hybrid machine. The gestures
 * used to control the KinectStick class are described in the
 * "Getting Started with the Microsoft Kinect for FRC" document
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	KinectStick leftArm;	//The Left arm should be constructed as stick 1
	KinectStick rightArm; 	//The Right arm should be constructed as stick 2
	Joystick stick;			//Joystick for teleop control

public:
	RobotDemo(void):
		myRobot(1, 2),	// these must be initialized in the same order
		leftArm(1),		// as they are declared above.
		rightArm(2),
		stick(1)
	{
		myRobot.SetExpiration(100000);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		bool exampleButton;
	
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "KinectStick2");
		dsLCD->UpdateLCD();
		GetWatchdog().SetEnabled(false);
		int counter =  0;
		/*A loop is necessary to retrieve the latest Kinect data and update the motors */
		while(IsAutonomous()){
            /**
             * KinectStick axis values are accessed identically to those of a joystick
             * In this example the axis values have been scaled by ~1/3 for safer
             * operation when learning to use the Kinect.
             */
			float left = leftArm.GetY();
			if(left >0){
				dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
						"left is positive");
			}
			else if(left <0){
				dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
						"left is negetive");
			}
            myRobot.TankDrive(leftArm.GetY(), rightArm.GetY());
			dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,
					"Left: %.5f", leftArm.GetY());
			dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,
					"Right: %.5f", rightArm.GetY());
			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,
					"%i", counter);
			dsLCD->UpdateLCD();
            /* An alternative illustrating that the KinectStick can be used just like a Joystick */
            //myRobot.TankDrive(leftArm, rightArm);

            /*Example illustrating that accessing buttons is identical to a Joystick */
            exampleButton = leftArm.GetRawButton(1);
            counter++;
            if(counter>= 1000){
            	counter = 0; 
            }
            Wait(.01); /* Delay 10ms to reduce processing load */
		}
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
			Wait(0.005);				// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

