
#include "MC36XX.h"
	
#ifdef MC36XX_CFG_BUS_I2C 
	#define MC36XX_CFG_I2C_ADDR		(0x4C)
#endif
#define MC36XX_CFG_MODE_DEFAULT			MC36XX_MODE_STANDBY
#define MC36XX_CFG_SAMPLE_RATE_CWAKE_DEFAULT    MC36XX_CWAKE_SR_DEFAULT_54Hz
#define MC36XX_CFG_SAMPLE_RATE_SNIFF_DEFAULT    MC36XX_SNIFF_SR_7Hz
#define MC36XX_CFG_RANGE_DEFAULT		MC36XX_RANGE_8G
#define MC36XX_CFG_RESOLUTION_DEFAULT		MC36XX_RESOLUTION_14BIT
#define MC36XX_CFG_ORIENTATION_MAP_DEFAULT    	ORIENTATION_TOP_RIGHT_UP

uint8_t CfgRange, CfgResolution;	

// Read register bit
bool MC36XX::readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = readRegister8(reg);
    return ((value >> pos) & 1);
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

// Read 8-bit from register
uint8_t MC36XX::readRegister8(uint8_t reg)
{
    uint8_t value;
	
#ifdef MC36XX_CFG_BUS_I2C 	
    Wire.beginTransmission(MC36XX_CFG_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false); 		//endTransmission but keep the connection active
    Wire.requestFrom(MC36XX_CFG_I2C_ADDR, 1); 	//Once done, bus is released by default
    value = Wire.read();
#else  //Reads an 8-bit register with the SPI port.
	digitalWrite(chipSelectPin, LOW); 	//Set active-low CS low to start the SPI cycle
	SPI.transfer(reg | 0x80 | 0x40); 	//Send the register address
	value = SPI.transfer(0x00); 		//Read the value from the register
	digitalWrite(chipSelectPin, HIGH); 	//Raise CS
#endif	

    return value;
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
	digitalWrite(chipSelectPin, LOW); 	//Set active-low CS low to start the SPI cycle
	SPI.transfer(reg | 0x40); 		//Send the register address
	SPI.transfer(value);  			//Send value to write into register
	digitalWrite(chipSelectPin, HIGH); 	//Raise CS
#endif	
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
	digitalWrite(chipSelectPin, LOW); 	//Set active-low CS low to start the SPI cycle
	SPI.transfer(reg | 0x80 | 0x40); 	//Send the register address
    uint8_t vha = SPI.transfer(0x00); 		//Send a value of 0 to read the first byte returned
    uint8_t vla = SPI.transfer(0x00); 		//Send a value of 0 to read the second byte returned
	digitalWrite(chipSelectPin, HIGH); 	//Raise CS
#endif		
	
	value = vha << 8 | vla;
	
    return value;
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
	digitalWrite(chipSelectPin, LOW); 		//Set active-low CS low to start the SPI cycle
	SPI.transfer(reg | 0x40); 			//Send the register address	
	SPI.transfer((uint8_t)(value >> 8));  		//Send value to write into register
	SPI.transfer((uint8_t)value);  			//Send value to write into register
	digitalWrite(chipSelectPin, HIGH); 		//Raise CS
#endif		
}

// Repeated Read Byte(s) from register
void MC36XX::readRegisters(uint8_t reg, byte *buffer, uint8_t len)
{
#ifdef MC36XX_CFG_BUS_I2C 	
    Wire.beginTransmission(MC36XX_CFG_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false); 			//endTransmission but keep the connection active
    Wire.requestFrom(MC36XX_CFG_I2C_ADDR, len); 	//Ask for bytes, once done, bus is released by default

    while(Wire.available() < len); 			//Hang out until we get the # of bytes we expect
    for(int x = 0 ; x < len ; x++)
		buffer[x] = Wire.read();
#else
	digitalWrite(chipSelectPin, LOW); 		//Set active-low CS low to start the SPI cycle
	SPI.transfer(reg | 0x80 | 0x40); 		//send the device the register you want to read
	
	for(int x = 0 ; x < len ; x++)              	//Prepare to clock in the data to be read
		buffer[x] = SPI.transfer(0x00);
	digitalWrite(chipSelectPin, HIGH); 		//Raise CS
#endif	

}
  
//Set the operation mode  
void MC36XX::SetMode(MC36XX_mode_t mode)
{
    uint8_t value;
    value = readRegister8(MC36XX_REG_MODE_C);
    value &= 0b11110000;
    value |= mode;
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

//Get the output sampling rate
MC36XX_cwake_sr_t MC36XX::GetCWakeSampleRate(void)
{
	/* Read the data format register to preserve bits */
	uint8_t value;
	value = readRegister8(MC36XX_REG_WAKE_C);
	Serial.println("GetCWakeSampleRate");
	Serial.println(value, HEX);			 
	value &= 0b00001111;
	return (MC36XX_cwake_sr_t) (value);
}

//Get the range control
MC36XX_range_t MC36XX::GetRangeCtrl(void)
{
	/* Read the data format register to preserve bits */
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
	/* Read the data format register to preserve bits */
	uint8_t value;
	value = readRegister8(MC36XX_REG_RANGE_C);
	Serial.println("GetResolutionCtrl");
	Serial.println(value, HEX);	 
	value &= 0x07;
	return (MC36XX_resolution_t) (value);
}

//Initialize the MC36XX sensor and set as the default configuration
bool MC36XX::start(void)
{
	
#ifdef MC36XX_CFG_BUS_I2C
	Wire.begin(); // Initialize I2C
#else
	digitalWrite(chipSelectPin, HIGH); //Set active-low CS low to start the SPI cycle	
	pinMode(chipSelectPin, OUTPUT);		
  #if 1
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  #else
	SPI.setDataMode (SPI_MODE3);
	SPI.setClockDivider(SPI_CLOCK_DIV8);
	SPI.begin(); // Initialize SPI
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
	SetRangeCtrl(MC36XX_RANGE_8G); 				//Range: 8g
	SetResolutionCtrl(MC36XX_RESOLUTION_14BIT); 		//Resolution: 14bit
	SetCWakeSampleRate(MC36XX_CWAKE_SR_DEFAULT_54Hz); 	//Sampling Rate: 50Hz
	SetMode(MC36XX_MODE_CWAKE); 				//Mode: Active
	  
	delay(50);	  
	  
	return true;
}

void MC36XX::stop()
{
	SetMode(MC36XX_MODE_SLEEP); //Set mode as Sleep
}

//Read the raw counts and SI units mearsurement data
MC36XX_acc_t MC36XX::readRawAccel(void)
{
	float faRange[5] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f}; //{2g, 4g, 8g, 16g, 12g}
	float faResolution[6] = { 32.0f, 64.0f, 128.0f, 512.0f, 2048.0f, 8192.0f}; //{6bit, 7bit, 8bit, 10bit, 12bit, 14bit}

	byte rawData[6];
	readRegisters(MC36XX_REG_XOUT_LSB, rawData, 6);  // Read the six raw data registers into data array
	x = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
	y = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
	z = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);

	AccRaw.XAxis = (short) (x);
	AccRaw.YAxis = (short) (y);
	AccRaw.ZAxis = (short) (z);
	AccRaw.XAxis_g = (float) (x) / faResolution[CfgResolution]*faRange[CfgRange];
	AccRaw.YAxis_g = (float) (y) / faResolution[CfgResolution]*faRange[CfgRange];
	AccRaw.ZAxis_g = (float) (z) / faResolution[CfgResolution]*faRange[CfgRange];

	return AccRaw;
}
