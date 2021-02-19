//=============================================================================
// Project   :  SHT3x Sample Code (V1.0)
// File      :  sht3x.c (V1.0)
// Author    :  jhshen@cs.ecnu.edu.cn
// Date      :  6/16/2015
// Controller:  STM32F103RC
// IDE       :  uVision V5.26
// Compiler  :  Armcc
// Brief     :  Sensor Layer: Implementation of functions for sensor access.
//=============================================================================

//-- Includes -----------------------------------------------------------------
#include "SHT3x.h"
#include "I2C_HAL.h"
//#include "stdio.h"

//-- Defines ------------------------------------------------------------------
// Generator polynomial for CRC
#define POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001


//==============================================================================
static uint8_t SHT3X_CalcCrc(uint8_t data[], uint8_t nbrOfBytes)
//==============================================================================
{
  uint8_t bit;        // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  uint8_t byteCtr;    // byte counter
  
  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
  {
    crc ^= (data[byteCtr]);
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else           crc = (crc << 1);
    }
  }
  
  return crc;
}

//==============================================================================
static uint8_t SHT3X_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
//==============================================================================
{
  uint8_t crc;     // calculated checksum
  
  // calculates 8-Bit checksum
  crc = SHT3X_CalcCrc(data, nbrOfBytes);
  
  // verify checksum
  if(crc != checksum) return 1;  // CHECKSUM_ERROR;
  else                return 0;
}

//===========================================================================
static int32_t SHT30_CalcTemperatureC(uint16_t u16sT)  // *100
//===========================================================================
{
   int32_t temperatureC;      // variable for result
   u16sT &= ~0x0003;         // clear bits [1..0] (status bits)
  /*
     * Formula T = -46.85 + 175.72 * ST / 2^16 from data sheet 6.2,
     * optimized for integer fixed point (3 digits) arithmetic
  */
  temperatureC = (((uint32_t)17572 * u16sT) >> 16) - 4685;
  return (int32_t)temperatureC;
}

//===========================================================================
static int32_t SHT30_CalcRH(uint16_t u16sRH)   // *100
//===========================================================================
{
  uint32_t humidityRH;       // variable for result
  u16sRH &= ~0x0003;         // clear bits [1..0] (status bits)
   /*
     * Formula RH = -6 + 125 * SRH / 2^16 from data sheet 6.1,
     * optimized for integer fixed point (3 digits) arithmetic
  */
   humidityRH = (((uint32_t)12500 * u16sRH) >> 16) - 600;
  return (int32_t)humidityRH;
}

/*
//==============================================================================
static float SHT3X_CalcTemperature(uint16_t rawValue)
//==============================================================================
{
  // calculate temperature
  // T = -45 + 175 * rawValue / (2^16-1)
  return 175.0f * (float)rawValue / 65535.0f - 45.0f;
}

//==============================================================================
static float SHT3X_CalcHumidity(uint16_t rawValue)
//==============================================================================
{
  // calculate relative humidity [%RH]
  // RH = rawValue / (2^16-1) * 100
  return 100.0f * (float)rawValue / 65535.0f;
}

//==============================================================================
static uint16_t SHT3X_CalcRawTemperature(float temperature)
//==============================================================================
{
  // calculate raw temperature [ticks]
  // rawT = (temperature + 45) / 175 * (2^16-1)
  return (temperature + 45.0f) / 175.0f * 65535.0f;
}


//==============================================================================
static uint16_t SHT3X_CalcRawHumidity(float humidity)
//==============================================================================
{
  // calculate raw relative humidity [ticks]
  // rawRH = humidity / 100 * (2^16-1)
  return humidity / 100.0f * 65535.0f;
}

*/

// SHT3X_ReadData(), Read and Calc temperature and humidity using command CMD_MEAS_CH(0x2C06)  with CRC
// output: update int32_t temperature and humidity if no error 
// return 0 if no error, return 1 if error
//==============================================================================
//uint8_t SHT3X_ReadData(float *mTemp, float *mHumi)   
uint8_t SHT3X_ReadData(int32_t *mTemp, int32_t *mHumi)   
//==============================================================================
{ 
	uint16_t comm = CMD_MEAS_CH;  // 0x2C06
	uint16_t tem, hum;
	uint8_t buff_t[3], buff_h[3];
	uint8_t checksum;
//	float Temperature=0;
//	float Humidity=0;
	
	I2C_Start();
	if ( I2C_WriteByte( ADDR_SHT30) == ACK)    // device addr
	  {
			if ( I2C_WriteByte( comm>>8 ) == ACK) 
			 {
				 if ( I2C_WriteByte( comm & 0x00ff ) == ACK) 
				 {
							I2C_Stop();
             	HAL_Delay(16);     // Measurement duration, 15ms Max.
            	I2C_Start();
	            if ( I2C_WriteByte( ADDR_SHT30 | 0x01) == ACK)  // read
                 {
		                buff_t[0]=I2C_ReadByte(ACK);
		                buff_t[1]=I2C_ReadByte(ACK);
		                buff_t[2]=I2C_ReadByte(ACK);    // temperature data CRC
		                buff_h[0]=I2C_ReadByte(ACK);
		                buff_h[1]=I2C_ReadByte(ACK);
		                buff_h[2]=I2C_ReadByte(NO_ACK); // RH data CRC
                   	I2C_Stop();
	               }
	
            	tem = (((uint16_t)buff_t[0]<<8) | buff_t[1]);  // temperatur data combine
            	hum = (((uint16_t)buff_h[0]<<8) | buff_h[1]);  // humidity data combine

              checksum = buff_t[2];
              if ( SHT3X_CheckCrc(buff_t, 2, checksum) ==0)
                  *mTemp = SHT30_CalcTemperatureC(tem);   // Calc Temp. and update

              checksum = buff_h[2];
              if ( SHT3X_CheckCrc(buff_h, 2, checksum) ==0)
                  *mHumi = SHT30_CalcRH(hum);   // Calc Temp. and update
		 
//	          	*mTemp = (175.0*(float)tem/65535.0-45.0) ;  // T = -45 + 175 * tem / (2^16-1)
//            	*mHumi = (100.0*(float)hum/65535.0);  // RH = hum*100 / (2^16-1)
	            return (0);
						
				 }
			 }
		}
//	I2C_Stop();	
	return (1);	 //error
}


// SHT3X_SetCommand, set 16 bits command of SHT3x
// return 0 if no error
//==============================================================================
uint8_t SHT3X_SetCommand(uint16_t comm)
//==============================================================================
{
	I2C_Start();
	if ( I2C_WriteByte( ADDR_SHT30) == ACK)    // device addr
	  {
			if ( I2C_WriteByte( comm>>8 ) == ACK)  // command high byte
			 {
				 if ( I2C_WriteByte( comm & 0x00ff ) == ACK) // command low byte
				 {
	          return (0);
				 }
			 }
		}
//	HAL_Delay(5);  // 50
//	I2C_Stop();	
	return (1);	 //error
}


// Readout of Measurement Results for Periodic Mode, command = 0xE000 with CRC
// output: update temperature and humidity if no error 
// return 0 if no error
//==============================================================================
uint8_t SHT3X_GetData(int32_t *iTemp, int32_t *iHumi)
//==============================================================================
{ uint16_t comm = 0xE000; 
	uint16_t tem, hum;
	uint8_t buff_t[3], buff_h[3];
	uint8_t checksum;
	
	I2C_Start();
	if ( I2C_WriteByte( ADDR_SHT30) == ACK)    // device addr
	  {
			if ( I2C_WriteByte( comm>>8 ) == ACK) 
			 {
				 if ( I2C_WriteByte( comm & 0x00ff ) == ACK) 
				 {
					 	I2C_Start();
	          if ( I2C_WriteByte( ADDR_SHT30 | 0x01) == ACK)  // read
                 {
		                buff_t[0]=I2C_ReadByte(ACK);
		                buff_t[1]=I2C_ReadByte(ACK);
		                buff_t[2]=I2C_ReadByte(ACK);    // temperature value CRC
		                buff_h[0]=I2C_ReadByte(ACK);
		                buff_h[1]=I2C_ReadByte(ACK);
		                buff_h[2]=I2C_ReadByte(NO_ACK); // RH value CRC
                   	I2C_Stop();
	               }
           	tem = (((uint16_t)buff_t[0]<<8) | buff_t[1]);  // temperatur data combine
           	hum = (((uint16_t)buff_h[0]<<8) | buff_h[1]);  // humidity data combine

            checksum = buff_t[2];
            if ( SHT3X_CheckCrc(buff_t, 2, checksum) ==0)
                  *iTemp = SHT30_CalcTemperatureC(tem);   // Calc Temp. and update

            checksum = buff_h[2];
            if ( SHT3X_CheckCrc(buff_h, 2, checksum) ==0)
                  *iHumi = SHT30_CalcRH(hum);   // Calc Temp. and update
            return (0);
				 }
			 }
		}
//	HAL_Delay(5);  // 50
//	I2C_Stop();	
	return (1);	 //error
}
