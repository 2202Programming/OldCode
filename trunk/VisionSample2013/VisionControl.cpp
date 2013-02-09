#include "VisionControl.h"
#include <cmath>

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

//Camera constants used for distance calculation
#define X_IMAGE_RES 320		//X Image resolution in pixels, should be 160, 320 or 640
#define VIEW_ANGLE 48		//Axis 206 camera
//#define VIEW_ANGLE 43.5  //Axis M1011 camera
#define PI 3.141592653

//Score limits used for target identification
#define RECTANGULARITY_LIMIT 60
#define ASPECT_RATIO_LIMIT 75
#define X_EDGE_LIMIT 40
#define Y_EDGE_LIMIT 60

//Minimum area of particles to be considered
#define AREA_MINIMUM 500

//Edge profile constants used for hollowness score calculation
#define XMAXSIZE 24
#define XMINSIZE 24
#define YMAXSIZE 24
#define YMINSIZE 48
const double xMax[XMAXSIZE] = { 1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5,
		.5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1 };
const double xMin[XMINSIZE] = { .4, .6, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1,
		.1, .1, .1, .1, .1, .1, .1, .1, .1, .1, 0.6, 0 };
const double yMax[YMAXSIZE] = { 1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5,
		.5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1 };
const double yMin[YMINSIZE] = { .4, .6, .05, .05, .05, .05, .05, .05, .05, .05,
		.05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
		.05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
		.05, .05, .05, .05, .05, .05, .05, .05, .6, 0 };

VisionControl::VisionControl() {

}

void VisionControl::initialize() {
	dsLCD = DriverStationLCD::GetInstance();
	smarty->init();

}
void VisionControl::initializeAutonomous() {

}
/*
 * returns distance to target in feet
 */
double VisionControl::run() {
	double distance = -1.0;
	double horizontalAngle = 100.0;
	double verticalAngle = 100.0;
	//bool isDone = false;
	//Threshold threshold(100, 125, 0, 255, 115, 255);
	//Threshold threshold(90, 140, 100, 255, 90, 255); //Lower hue, upper hue, lower saturation, higher saturation, lower lum, higher lum
	//Threshold threshold(60,112,0,255,34,100); //newJon
	Threshold threshold(70, 150, 150, 255, 160, 255);

	AxisCamera &camera = AxisCamera::GetInstance(); //To use the Axis camera uncomment this line
	ColorImage *image;
	//image = new RGBImage("/matthew2.bmp"); // get the sample image from the cRIO flash

	image = camera.GetImage();
	image->Write("Jon.bmp");
	BinaryImage *thresholdImage = image->ThresholdHSV(threshold); // get just the green target pixels
	BinaryImage *convexHullImage = thresholdImage->ConvexHull(false); // fill in partial and full rectangles
	//convexHullImage->Write("/ConvexH2ull.bmp");
	bool isOneParticle = false;
	if (convexHullImage->GetNumberParticles() < 1) {
		isOneParticle = true;
	}
	ParticleFilterCriteria2 criteria[] = { { IMAQ_MT_AREA, AREA_MINIMUM, 65535,
			false, isOneParticle } }; //Particle filter criteria, used to filter out small particles
	BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 1); //Remove small particles
	filteredImage->Write("/Filtered2.bmp");
	vector<ParticleAnalysisReport> *reports =
			filteredImage->GetOrderedParticleAnalysisReports(); //get a particle analysis report for each particle
	scores = new Scores[reports->size()];

	thresholdImage->Write("/threshold.bmp");
	smarty->PutNumber("Particlesfiltered", filteredImage->GetNumberParticles());
	smarty->PutNumber("ParticlesConvex", convexHullImage->GetNumberParticles());
	smarty->PutNumber("Report Size", reports->size());
	/*	smarty->PutNumber("Particlesfiltered", filteredImage->GetNumberParticles());
	 smarty->PutNumber("ParticlesConvex", convexHullImage->GetNumberParticles());
	 thresholdImage->Write("/Threshold.bmp");
	 convexHullImage->Write("/ConvexHull.bmp");
	 filteredImage->Write("/Filtered.bmp");

	 smarty->PutNumber("Timer:", timer.Get());
	 scoreTimer.Start();*/
	//Iterate through each particle, scoring it and determining whether it is a target or not
	for (unsigned i = 0; i < reports->size(); i++) {
		ParticleAnalysisReport *report = &(reports->at(i));

		scores[i].rectangularity = scoreRectangularity(report);
		scores[i].aspectRatioOuter = scoreAspectRatio(filteredImage, report,
				true);
		scores[i].aspectRatioInner = scoreAspectRatio(filteredImage, report,
				false);
		scores[i].aspectRatioLittle = scoreLittleAspectRatio(filteredImage,
				report);
		scores[i].xEdge = scoreXEdge(thresholdImage, report);
		scores[i].yEdge = scoreYEdge(thresholdImage, report);

		smarty->PutNumber("rectangularity", scores[i].rectangularity);
		smarty->PutNumber("aspectRatioLittle", scores[i].aspectRatioLittle);
		smarty->PutNumber("Xedge", scores[i].xEdge);
		smarty->PutNumber("Yedge", scores[i].yEdge);

		string goal = "not a rectangle";
		if (scoreCompare(scores[i], false)) {
			distance = computeDistance(thresholdImage, report, false);
			
			//			printf("particle: %d  is a High Goal  centerX: %f  centerY: %f \n",
			//					i, report->center_mass_x_normalized,
			//					report->center_mass_y_normalized);

			//			printf("Distance: %f \n",
			//					computeDistance(thresholdImage, report, false));
			//			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "dist: %f",
			//					computeDistance(filteredImage, report, false));
			//			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "high goal");
			//			dsLCD->UpdateLCD();
			i = reports->size();
			goal = "Center Goal";

		} else if (scoreCompare(scores[i], true)) {
			//			printf(
			//					"particle: %d  is a Middle Goal  centerX: %f  centerY: %f \n",
			//					i, report->center_mass_x_normalized,
			//					report->center_mass_y_normalized);
			//			printf("Distance: %f \n",
			//					computeDistance(thresholdImage, report, true));
			//			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "middist: %f",
			//					computeDistance(filteredImage, report, true));
			//			dsLCD->UpdateLCD();
			i = reports->size();
			goal = "Outer Goal";
		} else if (scoreCompare(scores[i], false, true)) { // Low Goal if
			distance = computeDistance(filteredImage, report, true, true);
			//			printf("particle: %d  is a low goal  centerX: %f  centerY: %f \n",
			//					i, report->center_mass_x_normalized,
			//					report->center_mass_y_normalized);
			
			//			printf("Distance: %f \n",
			//					computeDistance(filteredImage, report, true, true));
			//			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "lowdist: %f",
			//					computeDistance(filteredImage, report, true, true));
			//			dsLCD->UpdateLCD();
			/*	i = reports->size();
			 } else {
			 //			printf("particle: %d  is not a goal  centerX: %f  centerY: %f \n",
			 //					i, report->center_mass_x_normalized,
			 //					report->center_mass_y_normalized);
			 //			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3,
			 //					"Not a rectangle Goal");
			 //			dsLCD->UpdateLCD();
			 

			 */
			goal = "Little Goal";
			smarty->PutNumber("centerX", report->center_mass_x);
			smarty->PutNumber("centerXN", report->center_mass_x_normalized);
			smarty->PutNumber("centerY", report->center_mass_y);
			smarty->PutNumber("centerYN", report->center_mass_y_normalized);
		}

		//		printf("rect: %f  ARinner: %f \n", scores[i].rectangularity,
		//				scores[i].aspectRatioInner);
		//		printf("ARouter: %f  xEdge: %f  yEdge: %f  \n",
		//				scores[i].aspectRatioOuter, scores[i].xEdge, scores[i].yEdge);
		horizontalAngle = computeAngle(distance, report->center_mass_x_normalized);
		verticalAngle = computeAngle(distance, report->center_mass_y_normalized);
		smarty->PutNumber("Horizonal Angle", horizontalAngle);
		smarty->PutNumber("Vertical Angle", verticalAngle);
		smarty->PutString("Goal Type", goal);
	}

	// be sure to delete images after using them
	delete filteredImage;
	delete convexHullImage;
	delete thresholdImage;
	delete image;

	//delete allocated reports and Scores objects also
	delete scores;
	delete reports;
	//		
	return distance * 2.0;
}

bool VisionControl::runAuto() {
	return (true);
}

/*
 * This calculates the horizontal angle off center of the robot in respect to a target's center.
 * Parameters: Distance = distance from target, 
 * Be sure to use the normalized center value for this formula.
 * The formula returns the distance away from the center of the picture. 
 * Negative means that the angle is left/up of center, positive means that the angle is right/down of center. 
 */
double VisionControl::computeAngle(double distance, double centerNormal)
{
	double distanceOffCenter = 0.0; //the distance away from the origin of the picture
	double imageWidth = tan((VIEW_ANGLE/2.0)*(PI/180.0))*distance; // finds the half number of feet within the field of view at the given distance
	distanceOffCenter = (centerNormal*imageWidth);
	double angle = 0.0;
	angle = atan(distanceOffCenter/distance)*180.0/PI;
	return angle;
	
}


/**
 * Computes the estimated distance to a target using the height of the particle in the image. For more information and graphics
 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
 * returns in feet to target /2
 * @param image The image to use for measuring the particle estimated rectangle
 * @param report The Particle Analysis Report for the particle
 * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
 * @return The estimated distance to the target in Inches.
 */
double VisionControl::computeDistance(BinaryImage *image,
		ParticleAnalysisReport *report, bool outer, bool isLittle) {
	double rectShort, height;
	int targetHeight;

	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0,
			IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
	//using the smaller of the estimated rectangle short side and the bounding rectangle height results in better performance
	//on skewed rectangles
	height = min(report->boundingRect.height, rectShort);
	targetHeight = outer ? 29 : 20; // original is 20

	if (isLittle) {
		targetHeight = 32;
	}

	return X_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(
			VIEW_ANGLE * PI / (180 * 2)));
}

/**
 * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
 * the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed by moving
 * to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle area= x*y
 * and particle perimeter= 2x+2y
 * 
 * @param image The image containing the particle to score, needed to perform additional measurements
 * @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
 * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio for the inner target or the outer
 * @return The aspect ratio score (0-100)
 */
double VisionControl::scoreAspectRatio(BinaryImage *image,
		ParticleAnalysisReport *report, bool outer) {
	double rectLong, rectShort, idealAspectRatio, aspectRatio;
	idealAspectRatio = outer ? (62 / 29) : (62 / 20); //Dimensions of goal opening + 4 inches on all 4 sides for reflective tape

	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0,
			IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0,
			IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);

	//Divide width by height to measure aspect ratio
	if (report->boundingRect.width > report->boundingRect.height) {
		//particle is wider than it is tall, divide long by short
		aspectRatio = 100 * (1 - fabs(
				(1 - ((rectLong / rectShort) / idealAspectRatio))));
	} else {
		//particle is taller than it is wide, divide short by long
		aspectRatio = 100 * (1 - fabs(
				(1 - ((rectShort / rectLong) / idealAspectRatio))));
	}
	return (max(0, min(aspectRatio, 100))); //force to be in range 0-100
}

double VisionControl::scoreLittleAspectRatio(BinaryImage *image,
		ParticleAnalysisReport *report) {
	double rectLong, rectShort, idealAspectRatio, aspectRatio;
	idealAspectRatio = (37 / 32); // Dimensions of low goal opening 

	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0,
			IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0,
			IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);

	//Divide width by height to measure aspect ratio
	if (report->boundingRect.width > report->boundingRect.height) {
		//particle is wider than it is tall, divide long by short
		aspectRatio = 100 * (1 - fabs(
				(1 - ((rectLong / rectShort) / idealAspectRatio))));
	} else {
		//particle is taller than it is wide, divide short by long
		aspectRatio = 100 * (1 - fabs(
				(1 - ((rectShort / rectLong) / idealAspectRatio))));
	}
	return (max(0, min(aspectRatio, 100))); //force to be in range 0-100
}

/*
 
 * Compares scores to defined limits and returns true if the particle appears to be a target
 * 
 * @param scores The structure containing the scores to compare
 * @param outer True if the particle should be treated as an outer target, false to treat it as a center target, 
 * @return True if the particle meets all limits, false otherwise
 */
bool VisionControl::scoreCompare(Scores scores, bool outer, bool isLittle) {
	if (!isLittle) {
		bool isTarget = true;

		isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
		if (outer) {
			isTarget &= scores.aspectRatioOuter > ASPECT_RATIO_LIMIT;
		} else {
			isTarget &= scores.aspectRatioInner > ASPECT_RATIO_LIMIT;
		}
		isTarget &= scores.xEdge > X_EDGE_LIMIT;
		isTarget &= scores.yEdge > Y_EDGE_LIMIT;

		return isTarget;
	}
	bool isTarget = true;

	isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
	isTarget &= scores.aspectRatioLittle > ASPECT_RATIO_LIMIT;
	isTarget &= scores.xEdge > X_EDGE_LIMIT;
	isTarget &= scores.yEdge > Y_EDGE_LIMIT;

	return isTarget;

}

/**
 * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
 * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
 * 
 * @param report The Particle Analysis Report for the particle to score
 * @return The rectangularity score (0-100)
 */
double VisionControl::scoreRectangularity(ParticleAnalysisReport *report) {
	if (report->boundingRect.width * report->boundingRect.height != 0) {
		return 100 * report->particleArea / (report->boundingRect.width
				* report->boundingRect.height);
	} else {
		return 0;
	}
}
/**
 * Computes a score based on the match between a template profile and the particle profile in the X direction. This method uses the
 * the column averages and the profile defined at the top of the sample to look for the solid vertical edges with
 * a hollow center.
 * 
 * @param image The image to use, should be the image before the convex hull is performed
 * @param report The Particle Analysis Report for the particle
 * 
 * @return The X Edge Score (0-100)
 */
double VisionControl::scoreXEdge(BinaryImage *image,
		ParticleAnalysisReport *report) {
	double total = 0;
	LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(),
			IMAQ_COLUMN_AVERAGES, report->boundingRect);
	for (int i = 0; i < (averages->columnCount); i++) {
		if (xMin[i * (XMINSIZE - 1) / averages->columnCount]
				< averages->columnAverages[i] && averages->columnAverages[i]
				< xMax[i * (XMAXSIZE - 1) / averages->columnCount]) {
			total++;
		}
	}
	total = 100 * total / (averages->columnCount); //convert to score 0-100
	imaqDispose(averages); //let IMAQ dispose of the averages struct
	return total;
}

/**
 * Computes a score based on the match between a template profile and the particle profile in the Y direction. This method uses the
 * the row averages and the profile defined at the top of the sample to look for the solid horizontal edges with
 * a hollow center
 * 
 * @param image The image to use, should be the image before the convex hull is performed
 * @param report The Particle Analysis Report for the particle
 * 
 * @return The Y Edge score (0-100)
 */
double VisionControl::scoreYEdge(BinaryImage *image,
		ParticleAnalysisReport *report) {
	double total = 0;
	LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(),
			IMAQ_ROW_AVERAGES, report->boundingRect);
	for (int i = 0; i < (averages->rowCount); i++) {
		if (yMin[i * (YMINSIZE - 1) / averages->rowCount]
				< averages->rowAverages[i] && averages->rowAverages[i] < yMax[i
				* (YMAXSIZE - 1) / averages->rowCount]) {
			total++;
		}
	}
	total = 100 * total / (averages->rowCount); //convert to score 0-100
	imaqDispose(averages); //let IMAQ dispose of the averages struct
	return total;
}
