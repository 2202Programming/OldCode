/* Timothy robot,
 * Soaring disks throw to the goals.
 * Also, climbs. Maybe.
 */
#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"
#include "XboxController.h"
#include "DriveControl.h"

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

Threshold threshold(60, 100, 90, 255, 20, 255); //HSV threshold criteria, ranges are in that order ie. Hue is 60-100
ParticleFilterCriteria2 criteria[] = { { IMAQ_MT_AREA, AREA_MINIMUM,
		65535, false, false } }; //Particle filter criteria, used to filter out small particles

class Tim2013: public SimpleRobot {

#define AREA_MINIMUM 500
		struct Scores {
			double rectangularity;
			double aspectRatioInner;
			double aspectRatioOuter;
			double xEdge;
			double yEdge;
		};
		Scores *scores;	
	
	DriverStationLCD *dsLCD;
	DriveControl driveControl;
	// LiveWindow *window;
	AxisCamera *camera;
	//ColorImage *image;
	int cameraReturnSuccess;

public:
	Tim2013(void) {
		// window->LiveWindow::GetInstance(); // Not needed for now
		camera->AxisCamera::GetInstance("10.22.2.11");
		// window->AddSensor();
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Tim 2013 V 0.3");
		dsLCD->UpdateLCD();

	}
	void Autonomous(void) {

	}

	void OperatorControl(void) {
		driveControl.initialize();
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Hello Human");
						dsLCD->UpdateLCD();

		// Create Neccesary Vision Parameters
		ColorImage *image = NULL;

		//while (IsOperatorControl()) {
		// driveControl.run();
		/*cameraReturnSuccess = camera->GetImage(image);
		 
		 if (cameraReturnSuccess == 1){
		 dsLCD->PrintfLine(DriverStationLCD::kUser_Line2,
		 "Cam Working");
		 }
		 else{
		 dsLCD->PrintfLine(DriverStationLCD::kUser_Line2,
		 
		 "Cam Not Working");
		 }
		 */
		
		// Vision Processing Image Writing
		//image = new RGBImage("/testImage.jpg"); // to get image from cRIO flash
		// int captured; 
		// captured = camera->GetImage(image);		
		if (image != NULL) {
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Image Received!");

		} else {
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "No Image!");
		}
		BinaryImage *thresholdImage = image->ThresholdHSV(threshold); // get just the green target pixels
		thresholdImage->Write("/Threshold.bmp");

		BinaryImage *convexHullImage = thresholdImage->ConvexHull(false); // fill in partial and full rectangles
		convexHullImage->Write("/ConvexHull.bmp");

		BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria,
				1); //Remove small particles
		filteredImage->Write("/Filtered.bmp");
		dsLCD->UpdateLCD();

		vector<ParticleAnalysisReport> *reports =
				filteredImage->GetOrderedParticleAnalysisReports(); //get a particle analysis report for each particle
		scores = new Scores[reports->size()];

		//Iterate through each particle, scoring it and determining whether it is a target or not
		for (unsigned i = 0; i < reports->size(); i++) {

			ParticleAnalysisReport *report = &(reports->at(i));

			scores[i].rectangularity = scoreRectangularity(report);
			scores[i].aspectRatioOuter = scoreAspectRatio(filteredImage,
					report, true);
			scores[i].aspectRatioInner = scoreAspectRatio(filteredImage,
					report, false);
			scores[i].xEdge = scoreXEdge(thresholdImage, report);
			scores[i].yEdge = scoreYEdge(thresholdImage, report);

			dsLCD->PrintfLine(DriverStationLCD::kUser_Line5,
					"it got before if loop :(");
			dsLCD->UpdateLCD();
			if (scoreCompare(scores[i], false)) {
				printf(
						"particle: %d  is a High Goal  centerX: %f  centerY: %f \n",
						i, report->center_mass_x_normalized,
						report->center_mass_y_normalized);
				printf("Distance: %f \n",
						computeDistance(thresholdImage, report, false));
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "Distance %f",
						computeDistance(thresholdImage, report, false));
				dsLCD->UpdateLCD();
			} else if (scoreCompare(scores[i], true)) {
				printf(
						"particle: %d  is a Middle Goal  centerX: %f  centerY: %f \n",
						i, report->center_mass_x_normalized,
						report->center_mass_y_normalized);
				printf("Distance: %f \n",
						computeDistance(thresholdImage, report, true));
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "Distance %f",
						computeDistance(thresholdImage, report, true));
				dsLCD->UpdateLCD();
			} else {
				printf(
						"particle: %d  is not a goal  centerX: %f  centerY: %f \n",
						i, report->center_mass_x_normalized,
						report->center_mass_y_normalized);
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line3,
						"did not enter if statements");
				dsLCD->UpdateLCD();
			}
			printf("rect: %f  ARinner: %f \n", scores[i].rectangularity,
					scores[i].aspectRatioInner);
			printf("ARouter: %f  xEdge: %f  yEdge: %f  \n",
					scores[i].aspectRatioOuter, scores[i].xEdge,
					scores[i].yEdge);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "rect: %f",
					scores[i].rectangularity);
			dsLCD->UpdateLCD();

			Wait(0.005); // wait for a motor update time
		}
		//}
		
	}
	/**
		 * Computes the estimated distance to a target using the height of the particle in the image. For more information and graphics
		 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
		 * 
		 * @param image The image to use for measuring the particle estimated rectangle
		 * @param report The Particle Analysis Report for the particle
		 * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
		 * @return The estimated distance to the target in Inches.
		 */
		double computeDistance(BinaryImage *image, ParticleAnalysisReport *report,
				bool outer) {
			double rectShort, height;
			int targetHeight;

			imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0,
					IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
			//using the smaller of the estimated rectangle short side and the bounding rectangle height results in better performance
			//on skewed rectangles
			height = min(report->boundingRect.height, rectShort);
			targetHeight = outer ? 29 : 21;

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
		double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report,
				bool outer) {
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

		/**
		 * Compares scores to defined limits and returns true if the particle appears to be a target
		 * 
		 * @param scores The structure containing the scores to compare
		 * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
		 * 
		 * @return True if the particle meets all limits, false otherwise
		 */
		bool scoreCompare(Scores scores, bool outer) {
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

		/**
		 * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
		 * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
		 * 
		 * @param report The Particle Analysis Report for the particle to score
		 * @return The rectangularity score (0-100)
		 */
		double scoreRectangularity(ParticleAnalysisReport *report) {
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
		double scoreXEdge(BinaryImage *image, ParticleAnalysisReport *report) {
			double total = 0;
			LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(),
					IMAQ_COLUMN_AVERAGES, report->boundingRect);
			for (int i = 0; i < (averages->columnCount); i++) {
				if (xMin[i * (XMINSIZE - 1) / averages->columnCount]
						< averages->columnAverages[i]
						&& averages->columnAverages[i] < xMax[i * (XMAXSIZE - 1)
								/ averages->columnCount]) {
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
		double scoreYEdge(BinaryImage *image, ParticleAnalysisReport *report) {
			double total = 0;
			LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(),
					IMAQ_ROW_AVERAGES, report->boundingRect);
			for (int i = 0; i < (averages->rowCount); i++) {
				if (yMin[i * (YMINSIZE - 1) / averages->rowCount]
						< averages->rowAverages[i] && averages->rowAverages[i]
						< yMax[i * (YMAXSIZE - 1) / averages->rowCount]) {
					total++;
				}
			}
			total = 100 * total / (averages->rowCount); //convert to score 0-100
			imaqDispose(averages); //let IMAQ dispose of the averages struct
			return total;
		}
	};

	START_ROBOT_CLASS(Tim2013)
	;

