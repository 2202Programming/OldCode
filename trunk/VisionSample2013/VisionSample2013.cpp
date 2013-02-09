#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"
#include "VisionControl.h"
#include "XboxController.h"

/**
 * Sample program to use NIVision to find rectangles in the scene that are illuminated
 * by a LED ring light (similar to the model from FIRSTChoice). The camera sensitivity
 * is set very low so as to only show light sources and remove any distracting parts
 * of the image.
 * 
 * The CriteriaCollection is the set of criteria that is used to filter the set of
 * rectangles that are detected. In this example we're looking for rectangles with
 * a minimum width of 30 pixels and maximum of 400 pixels.
 * 
 * The algorithm first does a color threshold operation that only takes objects in the
 * scene that have a bright green color component. Then a convex hull operation fills 
 * all the rectangle outlines (even the partially occluded ones). Then a small object filter
 * removes small particles that might be caused by green reflection scattered from other 
 * parts of the scene. Finally all particles are scored on rectangularity, aspect ratio,
 * and hollowness to determine if they match the target.
 *
 * Look in the VisionImages directory inside the project that is created for the sample
 * images as well as the NI Vision Assistant file that contains the vision command
 * chain (open it with the Vision Assistant)
 */

class VisionSample2013: public SimpleRobot {

	SmartDashboard *smarty;
	RobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick
	VisionControl *visionControl;
	DriverStationLCD *dsLCD;
	XboxController *xbox;

public:
	VisionSample2013(void) :
		myRobot(1, 2), // these must be initialized in the same order
				stick(1) // as they are declared above.
	{
		xbox = XboxController::getInstance();
		smarty->init();
		visionControl = new VisionControl();
		myRobot.SetExpiration(0.1);
		myRobot.SetSafetyEnabled(false);
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Vision Sample 0.7");
		dsLCD->UpdateLCD();
	}

	/**
	 * Image processing code to identify 2013 Vision targets
	 */
	void Autonomous(void) {
		visionControl->initialize();
		int i = 1;
		float dist = 0.0;
		while (IsAutonomous() && IsEnabled()) {
			dist = visionControl->run();
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "ran %i", i);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "dist %f", dist);
			dsLCD->UpdateLCD();
			i++;
			Wait(0.5);
		}

	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void) {

		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl()) {
			Wait(0.005); // wait for a motor update time
		}
	}

};

START_ROBOT_CLASS(VisionSample2013)
;

