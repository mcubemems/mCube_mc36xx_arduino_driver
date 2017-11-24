
#include "MC36XX.h"

MC36XX MC36XX_acc = MC36XX();

void setup()
{
  	Serial.begin(115200);
    Serial.println("mCube Accelerometer MC36XX:");
    MC36XX_acc.start();
  	checkRange();
  	checkResolution();
  	checkSamplingRate();
  	Serial.println();
}

void checkRange()
{
	switch(MC36XX_acc.GetRangeCtrl())
	{
		case MC36XX_RANGE_16G:               Serial.println("Range: +/- 16 g"); break;
		case MC36XX_RANGE_12G:               Serial.println("Range: +/- 12 g"); break;
		case MC36XX_RANGE_8G:                Serial.println("Range: +/- 8 g"); break;
		case MC36XX_RANGE_4G:                Serial.println("Range: +/- 4 g"); break;
		case MC36XX_RANGE_2G:                Serial.println("Range: +/- 2 g"); break;
		default:                             Serial.println("Range: +/- 8 g"); break;
	}
}

void checkResolution()
{
	switch(MC36XX_acc.GetResolutionCtrl())
	{
		case MC36XX_RESOLUTION_6BIT:          Serial.println("Resolution: 6bit"); break;
		case MC36XX_RESOLUTION_7BIT:          Serial.println("Resolution: 7bit"); break;
		case MC36XX_RESOLUTION_8BIT:          Serial.println("Resolution: 8bit"); break;
		case MC36XX_RESOLUTION_10BIT:         Serial.println("Resolution: 10bit"); break;
		case MC36XX_RESOLUTION_14BIT:         Serial.println("Resolution: 14bit"); break;
		case MC36XX_RESOLUTION_12BIT:         Serial.println("Resolution: 12bit"); break;
		default:                              Serial.println("Resolution: 14bit"); break;
	}
}

void checkSamplingRate()
{
	Serial.println("Low Power Mode"); 
	switch(MC36XX_acc.GetCWakeSampleRate())
	{
		case MC36XX_CWAKE_SR_DEFAULT_54Hz:		Serial.println("Output Sampling Rate: 54 Hz"); break;
		case MC36XX_CWAKE_SR_14Hz:			Serial.println("Output Sampling Rate: 14 Hz"); break;
		case MC36XX_CWAKE_SR_28Hz:			Serial.println("Output Sampling Rate: 28 Hz"); break;
		case MC36XX_CWAKE_SR_54Hz:			Serial.println("Output Sampling Rate: 54 Hz"); break;
		case MC36XX_CWAKE_SR_105Hz:			Serial.println("Output Sampling Rate: 105 Hz"); break;
		case MC36XX_CWAKE_SR_210Hz:			Serial.println("Output Sampling Rate: 210 Hz"); break;
		case MC36XX_CWAKE_SR_400Hz:			Serial.println("Output Sampling Rate: 400 Hz"); break;
		case MC36XX_CWAKE_SR_600Hz:			Serial.println("Output Sampling Rate: 600 Hz"); break;
		default:					Serial.println("Output Sampling Rate: 54 Hz"); break;
	}
}

void loop()
{
	// Read the raw sensor data count
	MC36XX_acc_t rawAccel = MC36XX_acc.readRawAccel();
	delay(10);
	Serial.print("X:\t"); Serial.print(rawAccel.XAxis); Serial.print("\t");
	Serial.print("Y:\t"); Serial.print(rawAccel.YAxis); Serial.print("\t");
	Serial.print("Z:\t"); Serial.print(rawAccel.ZAxis); Serial.print("\t");
	Serial.println("counts");

	// Display the results (acceleration is measured in m/s^2)
	Serial.print("X: \t"); Serial.print(rawAccel.XAxis_g); Serial.print("\t");
	Serial.print("Y: \t"); Serial.print(rawAccel.YAxis_g); Serial.print("\t");
	Serial.print("Z: \t"); Serial.print(rawAccel.ZAxis_g); Serial.print("\t");
	Serial.println("m/s^2");

  Serial.println("--------------------------------------------------------------");
  delay(1000);
}
