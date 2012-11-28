/* This is the Header File for */
#ifndef SONARSENSOR_H
#define SONARSENSOR_H
#include "WPILib.h"
#include "Nivision.h"

class SonarSensor: public AnalogChannel{
public:
	SonarSensor(int channelInput);
	float getDistanceInInches();
	void setConversionRate(float newRate);
	
private:
	float conversionRate; //
	
};
#endif
