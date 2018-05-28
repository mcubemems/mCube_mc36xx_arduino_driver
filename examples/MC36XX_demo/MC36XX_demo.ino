/*****************************************************************************
 *
 * Copyright (c) 2018 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *****************************************************************************/

/**
 * @file    MC36XX_demo.ino
 * @author  mCube
 * @date    10 May 2018
 * @brief   Arduino example code for accelerometer mc36xx series.
 * @see     http://www.mcubemems.com
 */

#include "MC36XX.h"

#define INTERRUPT_PIN                8
#define FIFO_SIZE                    3

//DO NOT ENABLE (ENABLE_FIFO_WAKEUP) AND (ENABLE_SNIFF_SHAKE_WAKEUP) AT THE SAME TIME
#define ENABLE_FIFO_WAKEUP           0    //Sensor will wake up automatically when FIFO is full
#define ENABLE_SNIFF_SHAKE_WAKEUP    1    //Sensor will wake up until external force over the threshold setting

MC36XX_interrupt_event_t evt_mc36xx = {0};
MC36XX MC36XX_acc = MC36XX();

void setup()
{
    pinMode(INTERRUPT_PIN, INPUT);
    
    Serial.begin(115200);
    Serial.println("mCube Accelerometer MC36XX:");
    MC36XX_acc.start();
    checkRange();
    checkResolution();
    checkSamplingRate();
    checkSniffSamplingRate();
    Serial.println();

    //Test read
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

    Serial.println("---------------------------------------------------------");

#if ENABLE_SNIFF_SHAKE_WAKEUP
    sensorsniff();
#elif ENABLE_FIFO_WAKEUP
    sensorFIFO();
#endif
    
}

void sensorsniff()
{
    //Sensor sniff
    MC36XX_acc.stop();
    MC36XX_acc.SetSniffThreshold(MC36XX_AXIS_X,5);
    MC36XX_acc.SetSniffThreshold(MC36XX_AXIS_Y,5);
    MC36XX_acc.SetSniffThreshold(MC36XX_AXIS_Z,5);
    MC36XX_acc.SetSniffDetectCount(MC36XX_AXIS_X,3);
    MC36XX_acc.SetSniffDetectCount(MC36XX_AXIS_Y,3);
    MC36XX_acc.SetSniffDetectCount(MC36XX_AXIS_Z,3);
    MC36XX_acc.SetSniffAndOrN(MC36XX_ANDORN_OR);
    MC36XX_acc.SetSniffDeltaMode(MC36XX_DELTA_MODE_C2P);
    MC36XX_acc.SetINTCtrl(0,0,0,0,1); //Enable wake-up INT
    MC36XX_acc.sniff();
    Serial.println("Sensor sniff.");
}

void sensorFIFO()
{
    //Enable FIFO and interrupt
    MC36XX_acc.stop();
    MC36XX_acc.SetCWakeSampleRate(MC36XX_CWAKE_SR_14Hz);
    MC36XX_acc.SetResolutionCtrl(MC36XX_RESOLUTION_12BIT); //FIFO mode could only support 12 bit resolution
    MC36XX_acc.SetFIFOCtrl(MC36XX_FIFO_CTL_ENABLE, MC36XX_FIFO_MODE_WATERMARK, FIFO_SIZE);
    MC36XX_acc.SetINTCtrl(1,0,0,0,0); //Enable FIFO threshold interrupt
    MC36XX_acc.wake();
    
    Serial.println("Sensor FIFO enable.");
}

void checkRange()
{
    switch(MC36XX_acc.GetRangeCtrl())
    {
    case MC36XX_RANGE_16G:
        Serial.println("Range: +/- 16 g");
        break;
    case MC36XX_RANGE_12G:
        Serial.println("Range: +/- 12 g");
        break;
    case MC36XX_RANGE_8G:
        Serial.println("Range: +/- 8 g");
        break;
    case MC36XX_RANGE_4G:
        Serial.println("Range: +/- 4 g");
        break;
    case MC36XX_RANGE_2G:
        Serial.println("Range: +/- 2 g");
        break;
    default:
        Serial.println("Range: +/- 8 g");
        break;
    }
}

void checkResolution()
{
    switch(MC36XX_acc.GetResolutionCtrl())
    {
    case MC36XX_RESOLUTION_6BIT:
        Serial.println("Resolution: 6bit");
        break;
    case MC36XX_RESOLUTION_7BIT:
        Serial.println("Resolution: 7bit");
        break;
    case MC36XX_RESOLUTION_8BIT:
        Serial.println("Resolution: 8bit");
        break;
    case MC36XX_RESOLUTION_10BIT:
        Serial.println("Resolution: 10bit");
        break;
    case MC36XX_RESOLUTION_14BIT:
        Serial.println("Resolution: 14bit");
        break;
    case MC36XX_RESOLUTION_12BIT:
        Serial.println("Resolution: 12bit");
        break;
    default:
        Serial.println("Resolution: 14bit");
        break;
    }
}

void checkSamplingRate()
{
    Serial.println("Low Power Mode SR");
    switch(MC36XX_acc.GetCWakeSampleRate())
    {
    case MC36XX_CWAKE_SR_DEFAULT_54Hz:
        Serial.println("Output Sampling Rate: 54 Hz");
        break;
    case MC36XX_CWAKE_SR_14Hz:
        Serial.println("Output Sampling Rate: 14 Hz");
        break;
    case MC36XX_CWAKE_SR_28Hz:
        Serial.println("Output Sampling Rate: 28 Hz");
        break;
    case MC36XX_CWAKE_SR_54Hz:
        Serial.println("Output Sampling Rate: 54 Hz");
        break;
    case MC36XX_CWAKE_SR_105Hz:
        Serial.println("Output Sampling Rate: 105 Hz");
        break;
    case MC36XX_CWAKE_SR_210Hz:
        Serial.println("Output Sampling Rate: 210 Hz");
        break;
    case MC36XX_CWAKE_SR_400Hz:
        Serial.println("Output Sampling Rate: 400 Hz");
        break;
    case MC36XX_CWAKE_SR_600Hz:
        Serial.println("Output Sampling Rate: 600 Hz");
        break;
    default:
        Serial.println("Output Sampling Rate: 54 Hz");
        break;
    }
}

void checkSniffSamplingRate()
{
    Serial.println("Sniff Mode SR");
    switch(MC36XX_acc.GetSniffSampleRate())
    {
    case MC36XX_SNIFF_SR_DEFAULT_7Hz:
        Serial.println("Sniff Sampling Rate: 7 Hz");
        break;
    case MC36XX_SNIFF_SR_0p4Hz:
        Serial.println("Sniff Sampling Rate: 0.4 Hz");
        break;
    case MC36XX_SNIFF_SR_0p8Hz:
        Serial.println("Sniff Sampling Rate: 0.8 Hz");
        break;
    case MC36XX_SNIFF_SR_1p5Hz:
        Serial.println("Sniff Sampling Rate: 1.5 Hz");
        break;
    case MC36XX_SNIFF_SR_7Hz:
        Serial.println("Sniff Sampling Rate: 7 Hz");
        break;
    case MC36XX_SNIFF_SR_14Hz:
        Serial.println("Sniff Sampling Rate: 14 Hz");
        break;
    case MC36XX_SNIFF_SR_28Hz:
        Serial.println("Sniff Sampling Rate: 28 Hz");
        break;
    case MC36XX_SNIFF_SR_54Hz:
        Serial.println("Sniff Sampling Rate: 54 Hz");
        break;
    case MC36XX_SNIFF_SR_105Hz:
        Serial.println("Sniff Sampling Rate: 105 Hz");
        break;
    case MC36XX_SNIFF_SR_210Hz:
        Serial.println("Sniff Sampling Rate: 210 Hz");
        break;
    case MC36XX_SNIFF_SR_400Hz:
        Serial.println("Sniff Sampling Rate: 400 Hz");
        break;
    case MC36XX_SNIFF_SR_600Hz:
        Serial.println("Sniff Sampling Rate: 600 Hz");
        break;
    default:
        Serial.println("Sniff Sampling Rate: 7 Hz");
        break;
    }
}

void loop()
{
#if ENABLE_SNIFF_SHAKE_WAKEUP || ENABLE_FIFO_WAKEUP
    if (digitalRead(INTERRUPT_PIN) == HIGH)
    {
        Serial.println("Get interrupt.");
        delay(100);
#if ENABLE_FIFO_WAKEUP
        while(!(MC36XX_acc.IsFIFOEmpty()))
#endif
#endif
        {
            // Read the raw sensor data count
            MC36XX_acc_t rawAccel = MC36XX_acc.readRawAccel();
            
            Serial.print("X:\t"); Serial.print(rawAccel.XAxis); Serial.print("\t");
            Serial.print("Y:\t"); Serial.print(rawAccel.YAxis); Serial.print("\t");
            Serial.print("Z:\t"); Serial.print(rawAccel.ZAxis); Serial.print("\t");
            Serial.println("counts");
        
            // Display the results (acceleration is measured in m/s^2)
            Serial.print("X: \t"); Serial.print(rawAccel.XAxis_g); Serial.print("\t");
            Serial.print("Y: \t"); Serial.print(rawAccel.YAxis_g); Serial.print("\t");
            Serial.print("Z: \t"); Serial.print(rawAccel.ZAxis_g); Serial.print("\t");
            Serial.println("m/s^2");
        
            Serial.println("---------------------------------------------------------");

            rawAccel = MC36XX_acc.readRawAccel();
            
            Serial.print("X:\t"); Serial.print(rawAccel.XAxis); Serial.print("\t");
            Serial.print("Y:\t"); Serial.print(rawAccel.YAxis); Serial.print("\t");
            Serial.print("Z:\t"); Serial.print(rawAccel.ZAxis); Serial.print("\t");
            Serial.println("counts");
        
            // Display the results (acceleration is measured in m/s^2)
            Serial.print("X: \t"); Serial.print(rawAccel.XAxis_g); Serial.print("\t");
            Serial.print("Y: \t"); Serial.print(rawAccel.YAxis_g); Serial.print("\t");
            Serial.print("Z: \t"); Serial.print(rawAccel.ZAxis_g); Serial.print("\t");
            Serial.println("m/s^2");
        
            Serial.println("---------------------------------------------------------");
        }
#if ENABLE_SNIFF_SHAKE_WAKEUP || ENABLE_FIFO_WAKEUP
        MC36XX_acc.INTHandler(&evt_mc36xx);
#if ENABLE_SNIFF_SHAKE_WAKEUP
        sensorsniff();
#endif
    }
#else
    delay(1000);
#endif
}
