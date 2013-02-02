#ifndef VISONCONTROL_H
#define VISIONCONTROL_H

#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"


class VisionControl {
public:
	VisionControl();
	void initialize();
	void initializeAutonomous();
	void run();
	bool runAuto();

private:
	//Structure to represent the scores for the various tests used for target identification
	struct Scores {
		double rectangularity;
		double aspectRatioInner;
		double aspectRatioLittle;
		double aspectRatioOuter;
		double xEdge;
		double yEdge;

	};

	Scores *scores;
	DriverStationLCD *dsLCD;

	double computeDistance(BinaryImage *image, ParticleAnalysisReport *report, bool outer, bool isLittle = false);

	double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report,
			bool outer);
	double scoreLittleAspectRatio(BinaryImage *image,
				ParticleAnalysisReport *report);
	bool scoreCompare(Scores scores, bool outer, bool isLittle = false);
	double scoreRectangularity(ParticleAnalysisReport *report);
	double scoreXEdge(BinaryImage *image, ParticleAnalysisReport *report);
	double scoreYEdge(BinaryImage *image, ParticleAnalysisReport *report);
};

#endif
