/******************************************************************************
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
 * @file    MC36XX.c
 * @author  mCube
 * @date    10 May 2018
 * @brief   Driver interface header file for accelerometer mc36xx series.
 * @see     http://www.mcubemems.com
 */

#include "MC36XX.h"

#ifdef MC36XX_CFG_BUS_I2C
    #define MC36XX_CFG_I2C_ADDR        (0x4C)
#endif
#define MC36XX_CFG_MODE_DEFAULT                 MC36XX_MODE_STANDBY
#define MC36XX_CFG_SAMPLE_RATE_CWAKE_DEFAULT    MC36XX_CWAKE_SR_DEFAULT_54Hz
#define MC36XX_CFG_SAMPLE_RATE_SNIFF_DEFAULT    MC36XX_SNIFF_SR_105Hz//MC36XX_SNIFF_SR_DEFAULT_7Hz
#define MC36XX_CFG_RANGE_DEFAULT                MC36XX_RANGE_8G
#define MC36XX_CFG_RESOLUTION_DEFAULT           MC36XX_RESOLUTION_14BIT
#define MC36XX_CFG_ORIENTATION_MAP_DEFAULT      ORIENTATION_TOP_RIGHT_UP

uint8_t CfgRange, CfgResolution, CfgFifo, CfgINT;

// Read register bit
bool MC36XX::readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = readRegister8(reg);
    return ((value >> pos) & 1);
}

// Read 8-bit from register
uint8_t MC36XX::readRegister8(uint8_t reg)
{
    uint8_t value;

#ifdef MC36XX_CFG_BUS_I2C
    Wire.beginTransmission(MC36XX_CFG_I2C_ADDR);
    Wire.write(reg);
    //endTransmission but keep the connection active
    Wire.endTransmission(false);
    //Once done, bus is released by default
    Wire.requestFrom(MC36XX_CFG_I2C_ADDR, 1);
    value = Wire.read();
#else  //Reads an 8-bit register with the SPI port.
    //Set active-low CS low to start the SPI cycle
    digitalWrite(chipSelectPin, LOW);
    //Send the register address
    SPI.transfer(reg | 0x80 | 0x40);
    //Read the value from the register
    value = SPI.transfer(0x00);
    //Raise CS
    digitalWrite(chipSelectPin, HIGH);
#endif

    return value;
}

// Read 16-bit from register
int16_t MC36XX::readRegister16(uint8_t reg)
{
    int16_t value;

#ifdef MC36XX_CFG_BUS_I2C
    Wire.beginTransmission(MC36XX_CFG_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(MC36XX_CFG_I2C_ADDR, 2);
    while(!Wire.available()) {};
    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();
#else
    //Set active-low CS low to start the SPI cycle
    digitalWrite(chipSelectPin, LOW);
    //Send the register address
    SPI.transfer(reg | 0x80 | 0x40);
    //Send a value of 0 to read the first byte returned
    uint8_t vha = SPI.transfer(0x00);
    //Send a value of 0 to read the second byte returned
    uint8_t vla = SPI.transfer(0x00);
    //Raise CS
    digitalWrite(chipSelectPin, HIGH);
#endif

    value = vha << 8 | vla;

    return value;
}

// Repeated Read Byte(s) from register
void MC36XX::readRegisters(uint8_t reg, byte *buffer, uint8_t len)
{
#ifdef MC36XX_CFG_BUS_I2C
    Wire.beginTransmission(MC36XX_CFG_I2C_ADDR);
    Wire.write(reg);
    //endTransmission but keep the connection active
    Wire.endTransmission(false);
    //Ask for bytes, once done, bus is released by default
    Wire.requestFrom(MC36XX_CFG_I2C_ADDR, len);

    //Hang out until we get the # of bytes we expect
    while(Wire.available() < len);
    for(int x = 0 ; x < len ; x++)
        buffer[x] = Wire.read();
#else
    //Set active-low CS low to start the SPI cycle
    digitalWrite(chipSelectPin, LOW);
    //send the device the register you want to read
    SPI.transfer(reg | 0x80 | 0x40);

    //Prepare to clock in the data to be read
    for(int x = 0 ; x < len ; x++)
        buffer[x] = SPI.transfer(0x00);
    //Raise CS
    digitalWrite(chipSelectPin, HIGH);
#endif

}

// Write register bit
void MC36XX::writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readRegister8(reg);

    if (state)
        value |= (1 << pos);
    else
        value &= ~(1 << pos);

    writeRegister8(reg, value);
}

// Write 8-bit to register
void MC36XX::writeRegister8(uint8_t reg, uint8_t value)
{
#ifdef MC36XX_CFG_BUS_I2C
    Wire.beginTransmission(MC36XX_CFG_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
#else
    //Set active-low CS low to start the SPI cycle
    digitalWrite(chipSelectPin, LOW);
    //Send the register address
    SPI.transfer(reg | 0x40);
    //Send value to write into register
    SPI.transfer(value);
    //Raise CS
    digitalWrite(chipSelectPin, HIGH);
#endif
}

// Write 16-bit to register
void MC36XX::writeRegister16(uint8_t reg, int16_t value)
{
#ifdef MC36XX_CFG_BUS_I2C
    Wire.beginTransmission(MC36XX_CFG_I2C_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)value);
    Wire.endTransmission();
#else
    //Set active-low CS low to start the SPI cycle
    digitalWrite(chipSelectPin, LOW);
    //Send the register address
    SPI.transfer(reg | 0x40);
    //Send value to write into register
    SPI.transfer((uint8_t)(value >> 8));
    //Send value to write into register
    SPI.transfer((uint8_t)value);
    //Raise CS
    digitalWrite(chipSelectPin, HIGH);
#endif
}

//Initialize the MC36XX sensor and set as the default configuration
bool MC36XX::start(void)
{

#ifdef MC36XX_CFG_BUS_I2C
    // Initialize I2C
    Wire.begin();
#else
    //Set active-low CS low to start the SPI cycle
    digitalWrite(chipSelectPin, HIGH);
    pinMode(chipSelectPin, OUTPUT);
#if 1
	SPI.begin();
	SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
#else
    SPI.setDataMode (SPI_MODE3);
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    // Initialize SPI
    SPI.begin();
#endif
#endif

    //Init Reset
    reset();
    SetMode(MC36XX_MODE_STANDBY);

    //SetWakeAGAIN
    SetWakeAGAIN(MC36XX_GAIN_1X);
    //SetSniffAGAIN
    SetSniffAGAIN(MC36XX_GAIN_1X);

    /* Check I2C connection */
    uint8_t id = readRegister8(MC36XX_REG_PROD);
    if (id != 0x71)
    {
        /* No MC36XX detected ... return false */
        Serial.println("No MC36XX detected!");
        Serial.println(id, HEX);
        return false;
    }

//Running mc36xx in high speed SPI (8MHz)
#ifdef SPI_HS
    uint8_t value;
    value = readRegister8(MC36XX_REG_FEATURE_CTL);
    value &= 0b00111111;
    value |= 0x80 ;
    writeRegister8(MC36XX_REG_FEATURE_CTL, value);

    value = readRegister8(MC36XX_REG_PMCR);
    value |= 0x80 ;
    writeRegister8(MC36XX_REG_PMCR, value);
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE3));
    delay(50);
#endif

    //Range: 8g
    SetRangeCtrl(MC36XX_CFG_RANGE_DEFAULT);
    //Resolution: 14bit
    SetResolutionCtrl(MC36XX_CFG_RESOLUTION_DEFAULT);
    //Sampling Rate: 50Hz by default
    SetCWakeSampleRate(MC36XX_CFG_SAMPLE_RATE_CWAKE_DEFAULT);
	//Sampling Rate: 7Hz by default
    SetSniffSampleRate(MC36XX_CFG_SAMPLE_RATE_SNIFF_DEFAULT);
    //Mode: Active
    SetMode(MC36XX_MODE_CWAKE);

    delay(50);

    return true;
}

void MC36XX::wake()
{
    //Set mode as wake
    SetMode(MC36XX_MODE_CWAKE);
}

void MC36XX::stop()
{
    //Set mode as Sleep
    SetMode(MC36XX_MODE_STANDBY);
}

//Initial reset
void MC36XX::reset()
{
    writeRegister8(0x10, 0x01);

    delay(10);

    writeRegister8(0x24, 0x40);

    delay(50);

    writeRegister8(0x09, 0x00);
    delay(10);
    writeRegister8(0x0F, 0x42);
    delay(10);
    writeRegister8(0x20, 0x01);
    delay(10);
    writeRegister8(0x21, 0x80);
    delay(10);
    writeRegister8(0x28, 0x00);
    delay(10);
    writeRegister8(0x1a, 0x00);

    delay(50);

    uint8_t _bRegIO_C = 0;

    _bRegIO_C = readRegister8(0x0D);

    #ifdef MC36XX_CFG_BUS_I2C
        _bRegIO_C &= 0x3F;
        _bRegIO_C |= 0x40;
    #else
        _bRegIO_C &= 0x3F;
        _bRegIO_C |= 0x80;
    #endif

    writeRegister8(0x0D, _bRegIO_C);

    delay(50);

    writeRegister8(0x10, 0x01);

    delay(10);
}

void MC36XX::sniff()
{
    //Set mode as Sleep
    SetMode(MC36XX_MODE_SNIFF);
}

void MC36XX::sniffreset()
{
    uint8_t value;
	
    value = readRegister8(MC36XX_REG_SNIFF_CONF_C);
    value |= 0b10000000;
	
    writeRegister8(MC36XX_REG_SNIFF_CONF_C, value);
}

//Set the operation mode
void MC36XX::SetMode(MC36XX_mode_t mode)
{
    uint8_t value;
	uint8_t cfgfifovdd = 0x42;
	
    value = readRegister8(MC36XX_REG_MODE_C);
    value &= 0b11110000;
    value |= mode;
	
	writeRegister8(MC36XX_REG_PWR_CONTROL, cfgfifovdd);
    writeRegister8(MC36XX_REG_MODE_C, value);
}

//Set the range control
void MC36XX::SetRangeCtrl(MC36XX_range_t range)
{
    uint8_t value;
    CfgRange = range;
    SetMode(MC36XX_MODE_STANDBY);
    value = readRegister8(MC36XX_REG_RANGE_C);
    value &= 0b00000111;
    value |= (range << 4)&0x70 ;
    writeRegister8(MC36XX_REG_RANGE_C, value);
}

//Set the resolution control
void MC36XX::SetResolutionCtrl(MC36XX_resolution_t resolution)
{
    uint8_t value;
    CfgResolution = resolution;
    SetMode(MC36XX_MODE_STANDBY);
    value = readRegister8(MC36XX_REG_RANGE_C);
    value &= 0b01110000;
    value |= resolution;
    writeRegister8(MC36XX_REG_RANGE_C, value);
}

//Set the sampling rate
void MC36XX::SetCWakeSampleRate(MC36XX_cwake_sr_t sample_rate)
{
    uint8_t value;
    SetMode(MC36XX_MODE_STANDBY);
    value = readRegister8(MC36XX_REG_WAKE_C);
    value &= 0b00000000;
    value |= sample_rate;
    writeRegister8(MC36XX_REG_WAKE_C, value);
}

//Set the sniff sampling rate
void MC36XX::SetSniffSampleRate(MC36XX_sniff_sr_t sniff_sr)
{
    uint8_t value;
    SetMode(MC36XX_MODE_STANDBY);
    value = readRegister8(MC36XX_REG_SNIFF_C);
    value &= 0b00000000;
    value |= sniff_sr;
    writeRegister8(MC36XX_REG_SNIFF_C, value);
}

//Set FIFO
void MC36XX::SetFIFOCtrl(MC36XX_fifo_ctl_t fifo_ctl,
                         MC36XX_fifo_mode_t fifo_mode,
						 uint8_t fifo_thr)
{
    if (fifo_thr > 31)	//maximum threshold
        fifo_thr = 31;
		
    SetMode(MC36XX_MODE_STANDBY);
    
    CfgFifo = ((fifo_ctl << 6) | (fifo_mode << 5) | fifo_thr);
    writeRegister8(MC36XX_REG_FIFO_C, CfgFifo);
}

//Set interrupt control register
void MC36XX::SetINTCtrl(uint8_t fifo_thr_int_ctl,
                        uint8_t fifo_full_int_ctl,
						uint8_t fifo_empty_int_ctl,
						uint8_t acq_int_ctl,
						uint8_t wake_int_ctl)
{
		
    SetMode(MC36XX_MODE_STANDBY);
    
    CfgINT = (((fifo_thr_int_ctl & 0x01) << 6)
           | ((fifo_full_int_ctl & 0x01) << 5)
           | ((fifo_empty_int_ctl & 0x01) << 4)
           | ((acq_int_ctl & 0x01) << 3)
           | ((wake_int_ctl & 0x01) << 2)
           | MC36XX_INTR_C_IAH_ACTIVE_HIGH//MC36XX_INTR_C_IAH_ACTIVE_LOW//
           | MC36XX_INTR_C_IPP_MODE_PUSH_PULL);//MC36XX_INTR_C_IPP_MODE_OPEN_DRAIN);//
    writeRegister8(MC36XX_REG_INTR_C, CfgINT);
}

//Interrupt handler (clear interrupt flag)
void MC36XX::INTHandler(MC36XX_interrupt_event_t *ptINT_Event)
{
    uint8_t value;

    value = readRegister8(MC36XX_REG_STATUS_2);

    ptINT_Event->bWAKE           = ((value >> 2) & 0x01);
    ptINT_Event->bACQ            = ((value >> 3) & 0x01);
    ptINT_Event->bFIFO_EMPTY     = ((value >> 4) & 0x01);
    ptINT_Event->bFIFO_FULL      = ((value >> 5) & 0x01);
    ptINT_Event->bFIFO_THRESHOLD = ((value >> 6) & 0x01);
    ptINT_Event->bSWAKE_SNIFF    = ((value >> 7) & 0x01);
	
	value &= 0x03;
	writeRegister8(MC36XX_REG_STATUS_2, value);
}

//Set CWake Analog Gain
void MC36XX::SetWakeAGAIN(MC36XX_gain_t gain)
{
    writeRegister8(0x20, 0x01);
    uint8_t value;
    value = readRegister8(MC36XX_REG_GAIN);
    value &= 0b00111111;
    value |= (gain << 6);
    writeRegister8(MC36XX_REG_GAIN, value);
}

//Set Sniff Analog Gain
void MC36XX::SetSniffAGAIN(MC36XX_gain_t gain)
{
    writeRegister8(0x20, 0x00);
    uint8_t value;
    value = readRegister8(MC36XX_REG_GAIN);
    value &= 0b00111111;
    value |= (gain << 6);
    writeRegister8(MC36XX_REG_GAIN, value);
}

//Set Sniff threshold
void MC36XX::SetSniffThreshold(MC36XX_axis_t axis_cfg, uint8_t sniff_thr)
{
    uint8_t value;
	uint8_t regSniff_addr;
    value = readRegister8(MC36XX_REG_SNIFFTH_C);

    switch(axis_cfg)
    {
    case MC36XX_AXIS_X:
        regSniff_addr = 0x01; //Put X-axis to active
        break;
    case MC36XX_AXIS_Y: //Put Y-axis to active
        regSniff_addr = 0x02;
        break;
    case MC36XX_AXIS_Z: //Put Z-axis to active
        regSniff_addr = 0x03;
        break;
    default:
        break;
    }
	
	writeRegister8(MC36XX_REG_SNIFF_CONF_C, regSniff_addr);
    value |= sniff_thr;
    writeRegister8(MC36XX_REG_SNIFFTH_C, value);
}

//Set Sniff detect counts, 1~62 events
void MC36XX::SetSniffDetectCount(MC36XX_axis_t axis_cfg, uint8_t sniff_cnt)
{
    uint8_t value;
	uint8_t sniff_cfg;
	uint8_t regSniff_addr;
	
    sniff_cfg = readRegister8(MC36XX_REG_SNIFF_CONF_C);
	
    switch(axis_cfg)
    {
    case MC36XX_AXIS_X: //Select x detection count shadow register
        regSniff_addr = 0x05;
        break;
    case MC36XX_AXIS_Y: //Select y detection count shadow register
        regSniff_addr = 0x06;
        break;
    case MC36XX_AXIS_Z: //Select z detection count shadow register
        regSniff_addr = 0x07;
        break;
    default:
        break;
    }
	
	sniff_cfg |= regSniff_addr;
	writeRegister8(MC36XX_REG_SNIFF_CONF_C, sniff_cfg);
	
	value = readRegister8(MC36XX_REG_SNIFFTH_C);
	
    value |= sniff_cnt;
    writeRegister8(MC36XX_REG_SNIFFTH_C, value);
	
	sniff_cfg |= 0x08;
	writeRegister8(MC36XX_REG_SNIFF_CONF_C, sniff_cfg);
}

//Set sensor interrupt mode
void MC36XX::SetSniffAndOrN(MC36XX_andorn_t logicandor)
{
    uint8_t value;
	
    value = readRegister8(MC36XX_REG_SNIFFTH_C);
	
    switch(logicandor)
    {
    case MC36XX_ANDORN_OR:  //Axis or mode
        value &= 0xBF;
        break;
    case MC36XX_ANDORN_AND: //Axis and mode
        value |= 0x40;
        break;
    default:
        break;
    }
	
	writeRegister8(MC36XX_REG_SNIFFTH_C, value);
}

//Set sensor sniff delta mode
void MC36XX::SetSniffDeltaMode(MC36XX_delta_mode_t deltamode)
{
    uint8_t value;
	
    value = readRegister8(MC36XX_REG_SNIFFTH_C);
	
    switch(deltamode)
    {
    case MC36XX_DELTA_MODE_C2P: //Axis C2P mode
        value &= 0x7F;
        break;
    case MC36XX_DELTA_MODE_C2B: //Axis C2B mode
        value |= 0x80;
        break;
    default:
        break;
    }
	
	writeRegister8(MC36XX_REG_SNIFFTH_C, value);
	
    value = readRegister8(MC36XX_REG_SNIFFTH_C);
    Serial.println("SniffModeSet");
    Serial.println(value, HEX);
}

//Get the range control
MC36XX_range_t MC36XX::GetRangeCtrl(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = readRegister8(MC36XX_REG_RANGE_C);
    Serial.println("GetRangeCtrl");
    Serial.println(value, HEX);
    value &= 0x70;
    return (MC36XX_range_t) (value >> 4);
}

//Get the range control
MC36XX_resolution_t MC36XX::GetResolutionCtrl(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = readRegister8(MC36XX_REG_RANGE_C);
    Serial.println("GetResolutionCtrl");
    Serial.println(value, HEX);
    value &= 0x07;
    return (MC36XX_resolution_t) (value);
}

//Get the output sampling rate
MC36XX_cwake_sr_t MC36XX::GetCWakeSampleRate(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = readRegister8(MC36XX_REG_WAKE_C);
    Serial.println("GetCWakeSampleRate");
    Serial.println(value, HEX);
    value &= 0b00001111;
    return (MC36XX_cwake_sr_t) (value);
}

//Get the sniff sample rate
MC36XX_sniff_sr_t MC36XX::GetSniffSampleRate(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = readRegister8(MC36XX_REG_SNIFF_C);
    Serial.println("GetSniffSampleRate");
    Serial.println(value, HEX);
    value &= 0b00001111;
    return (MC36XX_sniff_sr_t) (value);
}

//Is FIFO empty
bool MC36XX::IsFIFOEmpty(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = readRegister8(MC36XX_REG_STATUS_1);
    value &= 0x10;
	Serial.println("FIFO_Status");
    Serial.println(value, HEX);
	
	if (value^0x10)
		return false;	//Not empty
	else
		return true;	//Is empty
}

//Read the raw counts and SI units measurement data
MC36XX_acc_t MC36XX::readRawAccel(void)
{
    //{2g, 4g, 8g, 16g, 12g}
    float faRange[5] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f};
    //{6bit, 7bit, 8bit, 10bit, 12bit, 14bit}
    float faResolution[6] = { 32.0f, 64.0f, 128.0f, 512.0f, 2048.0f, 8192.0f};

    byte rawData[6];
    // Read the six raw data registers into data array
    readRegisters(MC36XX_REG_XOUT_LSB, rawData, 6);
    x = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
    y = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
    z = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);

    AccRaw.XAxis = (short) (x);
    AccRaw.YAxis = (short) (y);
    AccRaw.ZAxis = (short) (z);
    AccRaw.XAxis_g = (float) (x)/faResolution[CfgResolution]*faRange[CfgRange];
    AccRaw.YAxis_g = (float) (y)/faResolution[CfgResolution]*faRange[CfgRange];
    AccRaw.ZAxis_g = (float) (z)/faResolution[CfgResolution]*faRange[CfgRange];

    return AccRaw;
}
