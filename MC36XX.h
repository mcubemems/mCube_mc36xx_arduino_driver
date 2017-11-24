#ifndef MC36XX_h
#define MC36XX_h

//#define MC36XX_CFG_BUS_I2C    // !!! DO NOT use both I2C and SPI simutaneously
#define MC36XX_CFG_BUS_SPI
/*
SS  : pin 2, Active-low CS¡Xchip select
MOSI: pin 11, MOSI¡Xmaster out slave in
MISO: pin 12, MISO¡Xmaster in slave out
SCK : pin 13, SCK - SPI clock
*/

#if (!defined (MC36XX_CFG_BUS_SPI) && !defined (MC36XX_CFG_BUS_I2C))
    #error "MUST use one bus to access register!"
#endif

#if (defined (MC36XX_CFG_BUS_SPI) && defined (MC36XX_CFG_BUS_I2C))
    #error "DO NOT use both SPI and I2C simutaneously!"
#endif

#ifdef MC36XX_CFG_BUS_I2C
	#include <Wire.h> 
#else
	#include <SPI.h>
	// pins used for the connection with the sensor
	// other information you can refer to the Arduino SPI library
	const int chipSelectPin = 2;
#endif 

#include "Arduino.h"

/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/
#define MC36XX_RETCODE_SUCCESS                 (0)
#define MC36XX_RETCODE_ERROR_BUS               (-1)
#define MC36XX_RETCODE_ERROR_NULL_POINTER      (-2)
#define MC36XX_RETCODE_ERROR_STATUS            (-3)
#define MC36XX_RETCODE_ERROR_SETUP             (-4)
#define MC36XX_RETCODE_ERROR_GET_DATA          (-5)
#define MC36XX_RETCODE_ERROR_IDENTIFICATION    (-6)
#define MC36XX_RETCODE_ERROR_NO_DATA           (-7)
#define MC36XX_RETCODE_ERROR_WRONG_ARGUMENT    (-8)
#define MC36XX_FIFO_DEPTH    					32
#define MC36XX_REG_MAP_SIZE    					64



/*******************************************************************************
 *** CONSTANT / DEFINE
 *******************************************************************************/

//=============================================
#define MC36XX_INTR_C_IPP_MODE_OPEN_DRAIN    (0x00)
#define MC36XX_INTR_C_IPP_MODE_PUSH_PULL     (0x01)

#define MC36XX_INTR_C_IAH_ACTIVE_LOW     	(0x00)
#define MC36XX_INTR_C_IAH_ACTIVE_HIGH    	(0x02)


/*******************************************************************************
 *** Register Map
 *******************************************************************************/
//=============================================
#define MC36XX_REG_EXT_STAT_1       (0x00)
#define MC36XX_REG_EXT_STAT_2       (0x01)
#define MC36XX_REG_XOUT_LSB         (0x02)
#define MC36XX_REG_XOUT_MSB         (0x03)
#define MC36XX_REG_YOUT_LSB         (0x04)
#define MC36XX_REG_YOUT_MSB         (0x05)
#define MC36XX_REG_ZOUT_LSB         (0x06)
#define MC36XX_REG_ZOUT_MSB         (0x07)
#define MC36XX_REG_STATUS_1         (0x08)
#define MC36XX_REG_STATUS_2         (0x09)
#define MC36XX_REG_MODE_C           (0x10)
#define MC36XX_REG_WAKE_C           (0x11)
#define MC36XX_REG_SNIFF_C          (0x12)
#define MC36XX_REG_SNIFFTH_C        (0x13)
#define MC36XX_REG_SNIFF_CONF_C     (0x14)
#define MC36XX_REG_RANGE_C          (0x15)
#define MC36XX_REG_FIFO_C           (0x16)
#define MC36XX_REG_INTR_C           (0x17)
#define MC36XX_REG_PROD             (0x18)
#define MC36XX_REG_DMX              (0x20)
#define MC36XX_REG_DMY              (0x21)
#define MC36XX_REG_GAIN             (0x21)
#define MC36XX_REG_DMZ              (0x22)
#define MC36XX_REG_RESET            (0x24)
#define MC36XX_REG_XOFFL            (0x2A)
#define MC36XX_REG_XOFFH            (0x2B)
#define MC36XX_REG_YOFFL            (0x2C)
#define MC36XX_REG_YOFFH            (0x2D)
#define MC36XX_REG_ZOFFL            (0x2E)
#define MC36XX_REG_ZOFFH            (0x2F)
#define MC36XX_REG_XGAIN            (0x30)
#define MC36XX_REG_YGAIN            (0x31)
#define MC36XX_REG_ZGAIN            (0x32)
#define MC36XX_REG_OPT              (0x3B)
#define MC36XX_REG_LOC_X            (0x3C)
#define MC36XX_REG_LOC_Y            (0x3D)
#define MC36XX_REG_LOT_dAOFSZ       (0x3E)
#define MC36XX_REG_WAF_LOT          (0x3F)

#define MC36XX_NULL_ADDR    		(0)

struct MC36XX_acc_t
{
    short XAxis;
    short YAxis;
    short ZAxis;      
    float XAxis_g;
    float YAxis_g;
    float ZAxis_g;
} ;

typedef enum
{
    MC36XX_GAIN_DEFAULT    = 0b00,
    MC36XX_GAIN_4X         = 0b01,
    MC36XX_GAIN_1X         = 0b10,
    MC36XX_GAIN_NOT_USED   = 0b11,   
}   MC36XX_gain_t;

typedef enum
{
    MC36XX_MODE_SLEEP 	   = 0b000,
    MC36XX_MODE_STANDBY    = 0b001,
    MC36XX_MODE_SNIFF      = 0b010,
    MC36XX_MODE_CWAKE      = 0b101, 
    MC36XX_MODE_TRIG       = 0b111,  
}   MC36XX_mode_t;

typedef enum
{
    MC36XX_RANGE_2G    = 0b000,
    MC36XX_RANGE_4G    = 0b001,
    MC36XX_RANGE_8G	   = 0b010,
    MC36XX_RANGE_16G   = 0b011,
    MC36XX_RANGE_12G   = 0b100,
    MC36XX_RANGE_END,
}   MC36XX_range_t;

typedef enum
{
    MC36XX_RESOLUTION_6BIT    = 0b000, 
    MC36XX_RESOLUTION_7BIT    = 0b001, 
    MC36XX_RESOLUTION_8BIT    = 0b010, 
    MC36XX_RESOLUTION_10BIT   = 0b011, 
    MC36XX_RESOLUTION_12BIT   = 0b100, 
    MC36XX_RESOLUTION_14BIT   = 0b101,  //(Do not select if FIFO enabled)
    MC36XX_RESOLUTION_END,
}   MC36XX_resolution_t;

typedef enum
{
    MC36XX_CWAKE_SR_DEFAULT_54Hz = 0b0000,
    MC36XX_CWAKE_SR_14Hz         = 0b0101,
    MC36XX_CWAKE_SR_28Hz         = 0b0110,
    MC36XX_CWAKE_SR_54Hz         = 0b0111,
    MC36XX_CWAKE_SR_105Hz        = 0b1000,
    MC36XX_CWAKE_SR_210Hz        = 0b1001,
    MC36XX_CWAKE_SR_400Hz        = 0b1010,
    MC36XX_CWAKE_SR_600Hz        = 0b1011,
    MC36XX_CWAKE_SR_END,
}   MC36XX_cwake_sr_t;

typedef enum
{
    MC36XX_SNIFF_SR_DEFAULT_7Hz = 0b0000,
    MC36XX_SNIFF_SR_0p4Hz       = 0b0001,
    MC36XX_SNIFF_SR_0p8Hz       = 0b0010,
    MC36XX_SNIFF_SR_1p5Hz       = 0b0011,
    MC36XX_SNIFF_SR_7Hz         = 0b0100,
    MC36XX_SNIFF_SR_14Hz        = 0b0101,
    MC36XX_SNIFF_SR_28Hz        = 0b0110,
    MC36XX_SNIFF_SR_54Hz        = 0b0111,
    MC36XX_SNIFF_SR_105Hz       = 0b1000,
    MC36XX_SNIFF_SR_210Hz       = 0b1001,
    MC36XX_SNIFF_SR_400Hz       = 0b1010,
    MC36XX_SNIFF_SR_600Hz       = 0b1011,
    MC36XX_SNIFF_SR_END,
}   MC36XX_sniff_sr_t;

typedef enum
{
    MC36XX_FIFO_CONTROL_DISABLE = 0,
    MC36XX_FIFO_CONTROL_ENABLE,
    MC36XX_FIFO_CONTROL_END,
}   MC36XX_fifo_control_t;

typedef enum
{
    MC36XX_FIFO_MODE_NORMAL = 0,
    MC36XX_FIFO_MODE_WATERMARK,
    MC36XX_FIFO_MODE_END,
}   MC36XX_fifo_mode_t;

typedef struct
{
    unsigned char    bWAKE;              // Sensor wakes from sniff mode.
    unsigned char    bACQ;               // New sample is ready and acquired.
    unsigned char    bFIFO_EMPTY;        // FIFO is empty.
    unsigned char    bFIFO_FULL;         // FIFO is full.
    unsigned char    bFIFO_THRESHOLD;    // FIFO sample count is equal to or greater than the threshold count.
    unsigned char    bRESV;
    unsigned char    baPadding[2];
}   MC36XX_InterruptEvent;


class MC36XX{
 public:

  /* general accel methods */
  	bool start();      // begin measurements
  	void stop();       // end measurments
	void reset();
  	void SetMode(MC36XX_mode_t);
	void SetRangeCtrl(MC36XX_range_t);   
	void SetResolutionCtrl(MC36XX_resolution_t); 
	void SetCWakeSampleRate(MC36XX_cwake_sr_t);
	void SetSniffAGAIN(MC36XX_gain_t);
	void SetWakeAGAIN(MC36XX_gain_t);
	MC36XX_sniff_sr_t GetSniffSampleRate(MC36XX_sniff_sr_t);
	MC36XX_resolution_t GetResolutionCtrl(void);	
	MC36XX_range_t GetRangeCtrl(void);
	MC36XX_cwake_sr_t GetCWakeSampleRate(void);   
	MC36XX_acc_t readRawAccel(void);

	
 private:
 	short x, y, z;
	MC36XX_acc_t AccRaw; // Raw Accelerometer data
	uint8_t readRegister8(uint8_t reg);
	int16_t readRegister16(uint8_t reg);
	void readRegisters(uint8_t reg, byte *buffer, uint8_t len);  
	bool readRegisterBit(uint8_t reg, uint8_t pos);
	void writeRegisterBit(uint8_t reg, uint8_t pos, bool state);
	void writeRegister16(uint8_t reg, int16_t value);
	void writeRegister8(uint8_t reg, uint8_t value);       
};
#endif
