#ifndef SHT2x_H
#define SHT2x_H

#include "main.h"

#define ADDR_SHT20    0x80    // SHT20 device addr, 0x40<<1
#define CMD_RH        0xF5    // read RH command for SHT20
#define CMD_TEM       0xF3    // read Temperature for SHT20

//P(x)=x^8+x^5+x^4+1 = 100110001, STH20 CRC 
#define CRC_POLY  0x131
 
/******************************************************************************
* C++ DECLARATION WRAPPER
******************************************************************************/
 
#ifdef __cplusplus
extern "C" {
#endif
 
/******************************************************************************
* EXPORTED MACROS AND DEFINITIONS
******************************************************************************/ 

// Convert Float and Char
typedef union 
{
	float x;
	uint8_t y[4];
}FloatChar;


typedef enum SHT20Resolution_t
{
  SHT2x_RES_12_14BIT       = 0x00, // RH=12bit, T=14bit
  SHT2x_RES_8_12BIT        = 0x01, // RH= 8bit, T=12bit
  SHT2x_RES_10_13BIT       = 0x80, // RH=10bit, T=13bit
  SHT2x_RES_11_11BIT       = 0x81, // RH=11bit, T=11bit
  SHT2x_RES_MASK           = 0x81  // Mask for res. bits (7,0) in user reg.
} SHT20Resolution_t;
 
typedef enum SHT20StatusCode {
  SHT2x_STATUS_OK                = 0x00,
  SHT2x_STATUS_VALID_DATA        = 0x01,
  SHT2x_STATUS_NO_CHANGE         = 0x02,
  SHT2x_STATUS_ABORTED           = 0x03,
  SHT2x_STATUS_BUSY              = 0x04,
  SHT2x_STATUS_SUSPEND           = 0x05,
  SHT2x_STATUS_ERR_IO            = 0x06,
  SHT2x_STATUS_ERR_BAD_DATA      = 0x07,
  SHT2x_STATUS_ERR_TIMEOUT       = 0x08
}SHT20StatusCode;
 
// sensor command
typedef enum{
  TRIG_T_MEASUREMENT_HM    = 0xE3, // command trig. temp meas. hold master
  TRIG_RH_MEASUREMENT_HM   = 0xE5, // command trig. humidity meas. hold master
  TRIG_T_MEASUREMENT_POLL  = 0xF3, // command trig. temp meas. no hold master
  TRIG_RH_MEASUREMENT_POLL = 0xF5, // command trig. humidity meas. no hold master
  USER_REG_W               = 0xE6, // command writing user register
  USER_REG_R               = 0xE7, // command reading user register
  SOFT_RESET               = 0xFE  // command soft reset
}SHT20Command_t;
 
typedef enum {
  SHT2x_EOB_ON             = 0x40, // end of battery
  SHT2x_EOB_MASK           = 0x40, // Mask for EOB bit(6) in user reg.
} SHT20Eob_t;
 
typedef enum {
  SHT2x_HEATER_ON          = 0x04, // heater on
  SHT2x_HEATER_OFF         = 0x00, // heater off
  SHT2x_HEATER_MASK        = 0x04, // Mask for Heater bit(2) in user reg.
} etSHT2xHeater;
 
// measurement signal selection
typedef enum{
  HUMIDITY,
  TEMP
}etSHT2xMeasureType;
 
typedef enum{
  I2C_ADR_W                = 0x80,   // sensor SHA20 I2C address + write bit
  I2C_ADR_R                = 0x81    // sensor SHA20 I2C address + read bit
}etI2cHeader;
 
/******************************************************************************
* EXPORTED FUNCTIONS
******************************************************************************/
uint8_t SHT20_SoftReset( void );
uint8_t SHT20_GetBatteryStatus(void);
uint8_t SHT20_GetHeaterStatus(void);
uint8_t SHT20_GetResolution(SHT20Resolution_t *pResolution);

uint8_t  SHT20_Init( void );
uint8_t  SHT20_GetTemp( int32_t *pMeasurand );
uint8_t  SHT20_GetRH( int32_t *pMeasurand );
float    ReadSHT20(uint8_t command);

/******************************************************************************
* END OF C++ DECLARATION WRAPPER
******************************************************************************/
 
#ifdef __cplusplus
}
#endif
 
#endif
