/* This is the CPP file for Xbox Controller */

#include "WPILib.h"
#include "SonarSensor.h"

#define DEFAULT_CONVERSION_RATE 9.8

SonarSensor::SonarSensor(int channelInput): AnalogChannel(channelInput){
	setConversionRate(DEFAULT_CONVERSION_RATE);
}

void SonarSensor::setConversionRate(float newRate){
	conversionRate=newRate;
}

//returns distance in inches
float SonarSensor::getDistanceInInches(){
	float voltage=GetAverageVoltage();
	return (voltage)/(5.01/512.0);
}

