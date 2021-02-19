
/*******************************************************************************
* EXPORT INCLUDE FILES
*******************************************************************************/

//#include "main.h"
#include "I2C_HAL.h"  

/*******************************************************************************
* LOCAL INCLUDE FILES
*******************************************************************************/
#include "SHT2x.h"

/******************************************************************************
* LOCAL MACROS AND DEFINITIONS
******************************************************************************/

/******************************************************************************
* LOCAL FUNCTION DECLARATIONS
******************************************************************************/
//static void i2c_Delay(uint16_t value );
static uint8_t SHT20_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);
static int32_t SHT20_CalcTemperatureC(uint16_t u16sT);
static int32_t SHT20_CalcRH(uint16_t u16sRH);
 
//===========================================================================
static uint8_t SHT20_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
//===========================================================================
{
    uint8_t crc = 0;
    uint8_t byteCtr;
    
    //calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
    {
      crc ^= (data[byteCtr]);
      for (uint8_t bit = 8; bit > 0; --bit)
      {
        if (crc & 0x80)  crc = (crc << 1) ^ CRC_POLY;
                  else   crc = (crc << 1);
        
      }
    }
 
   if (crc != checksum) return (1);
         else return (0);
}
 

//===========================================================================
uint8_t SHT20_ReadUserRegister(uint8_t *pRegisterValue)
//===========================================================================
{
  uint8_t checksum;   //variable for checksum byte
  uint8_t error=0;    //variable for error code

  I2C_Start();
  error |= I2C_WriteByte (ADDR_SHT20);
  error |= I2C_WriteByte (USER_REG_R);
  I2C_Start();
  error |= I2C_WriteByte (ADDR_SHT20 | 0x01);
  *pRegisterValue = I2C_ReadByte(ACK);
  checksum=I2C_ReadByte(NO_ACK);
  error |= SHT20_CheckCrc (pRegisterValue,1,checksum);
  I2C_Stop();
  return error;
}

//===========================================================================
uint8_t SHT20_WriteUserRegister(uint8_t *pRegisterValue)
//===========================================================================
{
  uint8_t error=0;   //variable for error code

  I2C_Start();
  error |= I2C_WriteByte (I2C_ADR_W);
  error |= I2C_WriteByte (USER_REG_W);
  error |= I2C_WriteByte (*pRegisterValue);
  I2C_Stop();
  return error;
}

//===========================================================================
static int32_t SHT20_CalcTemperatureC(uint16_t u16sT)
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
static int32_t SHT20_CalcRH(uint16_t u16sRH)
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

// SHT20_GetTemp, read and calc Temperature with CRC
// return 0 if no error, do update int32 temperature value
// return 0x08 if error, don't update humidity value
//===========================================================================
uint8_t  SHT20_GetTemp( int32_t *pMeasurand )
//===========================================================================
{
    uint8_t ret = SHT2x_STATUS_OK;    // 0x00
    uint8_t checksum;                 //checksum
    uint16_t rawdata;
    uint8_t cmd = 0xF3;  // temperature
    uint8_t data[3] = {0, 0, 0};        //data array for checksum v
    
    I2C_Start();
    if ( I2C_WriteByte( ADDR_SHT20) == ACK)  // Device address 
		{
      if ( I2C_WriteByte( cmd) == ACK)
			{
		    do{
		   	    HAL_Delay(1);
		        I2C_Start(); 
			    } while(I2C_WriteByte(ADDR_SHT20 | 0x01) == NO_ACK);   // polling read status
        data[0] = I2C_ReadByte(ACK);   // read data
        data[1] = I2C_ReadByte(ACK);
        data[2] = I2C_ReadByte(NO_ACK);  // read check sum
        I2C_Stop();
        rawdata = ((uint16_t)data[0] << 8) | data[1];  // uint16
        checksum = data[2];
 
        ret = SHT20_CheckCrc(data, 2, checksum);   
        if (ret == SHT2x_STATUS_OK)   // CRC ok?
                 *pMeasurand = SHT20_CalcTemperatureC(rawdata);  // Calc Temp. and return
//            else *pMeasurand = rawdata;  // return raw data
        return ret;  // no error
			}
		}			
//    I2C_Stop();
   ret = SHT2x_STATUS_ERR_TIMEOUT ;  // timeout 0x08
   return ret;
}
 
/******************************************************************************
* EXPORTED FUNCTIONS
******************************************************************************/

// SHT20_GetRH, read and calc Humidity with CRC
// return 0 if no error, do update int32 humidity value
// return 0x08 if error, don't update humidity value
//===========================================================================
uint8_t  SHT20_GetRH( int32_t *pMeasurand )
//===========================================================================
{
    uint8_t ret = SHT2x_STATUS_OK;  // 0x00
    uint8_t checksum;   //checksum
    uint16_t rawdata;
    uint8_t cmd = 0xF5;  // TRIG_RH_MEASUREMENT_POLL;
    uint8_t data[3] = {0, 0, 0}; //data array for checksum v
    
    I2C_Start();
    if ( I2C_WriteByte( ADDR_SHT20 | 0x00 ) == ACK )  // Device address 
		 {  if ( I2C_WriteByte( cmd ) == ACK )  
			   {
 		       do{
		   	       HAL_Delay(1);
			         I2C_Start(); 
			       } while(I2C_WriteByte(ADDR_SHT20 | 0x01) == NO_ACK);   // polling read status
           data[0] = I2C_ReadByte(ACK);  // read data
           data[1] = I2C_ReadByte(ACK);
           data[2] = I2C_ReadByte(NO_ACK);  // read check sum
           I2C_Stop();    
           rawdata = ((uint16_t)data[0] << 8) | data[1];
           checksum = data[2];
           ret = SHT20_CheckCrc(data, 2, checksum);   
           if (ret == SHT2x_STATUS_OK)
                  *pMeasurand = SHT20_CalcRH(rawdata);  // Calc RH & return
 //            else *pMeasurand = rawdata;  // return raw data
           return ret;   // no error
				 }
		 }	 
//   I2C_Stop();
   ret = SHT2x_STATUS_ERR_TIMEOUT;  // timeout 0x08
   return ret;
}
 
//===========================================================================
uint8_t  SHT20_SoftReset( void )
//===========================================================================
{
    uint8_t ret = 0;  // SHT2x_STATUS_OK;
    uint8_t cmd = 0xFE;   // SOFT_RESET;
    
    I2C_Start();
    if ( I2C_WriteByte( ADDR_SHT20 ) == ACK )  // Device address 
		  {
        if ( I2C_WriteByte( cmd ) == ACK )
				{					
	        I2C_Stop();     // 
	        HAL_Delay(15);  // soft reset take 15ms 
          return ret;
				}
			}				
    I2C_Stop();
    ret = 0x08;  // SHT2x_STATUS_ERR_TIMEOUT;
    return ret;
}

//===========================================================================
uint8_t  SHT20_GetBatteryStatus(void)
//===========================================================================
{
  uint8_t reg;
  uint8_t error = SHT2x_STATUS_OK;
 
  error = SHT20_ReadUserRegister(&reg);
 
  if (error != SHT2x_STATUS_OK)
  {
    return error;
  }
 
  return (reg & 0x40);
}
 
//===========================================================================
uint8_t SHT20_GetHeaterStatus(void)
//===========================================================================
{
  uint8_t reg;
  uint8_t error = SHT2x_STATUS_OK;
 
  error = SHT20_ReadUserRegister(&reg);
 
  if (error != SHT2x_STATUS_OK)
  {
    return error;
  }
 
  return (reg & 0x04);
}
 
//===========================================================================
uint8_t SHT20_GetResolution(SHT20Resolution_t *pResolution)
//===========================================================================
{
    uint8_t error = SHT2x_STATUS_OK;
    uint8_t reg = 0;
 
    error = SHT20_ReadUserRegister(&reg);
    if (error != SHT2x_STATUS_OK)
      {
        return error;
      }
    *pResolution = (SHT20Resolution_t)(reg & SHT2x_RES_MASK);
    return error;
}

// SHT20_Init(), set user register with Resolution e.g. RH 10bit, Temp 13bit
// return 0 if no error
//===========================================================================
uint8_t SHT20_Init( void )
//===========================================================================
{   uint8_t userRegister, error=0;
//    SHT20Resolution_t SHT20Resolutionvalue;
	
//	  error |= SHT20_SoftReset();
	    // --- Set Resolution e.g. RH 10bit, Temp 13bit ---
    error |= SHT20_ReadUserRegister(&userRegister);  //get actual user reg
    userRegister = (userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_10_13BIT;
    error |= SHT20_WriteUserRegister(&userRegister); //write changed user reg

// 	  HAL_Delay(10);
//    error |= SHT20_GetBatteryStatus();   
//    error |= SHT20_GetResolution( &SHT20Resolutionvalue );
	  return error;
}

// ReadSHT20, simple test Temperature or RH, no CRC check, 
// command = CMD_RH(0xF5) / CMD_TEM(0xF3)
// return float temperature / Humidity, return 0 if error
//================================================================================
float ReadSHT20(uint8_t command)   
//================================================================================
{
	float temp;
	uint8_t MSB,LSB;
	float Humidity;
	float Temperature;
 
//	SET_Resolution();
	I2C_Start(); 
	if(I2C_WriteByte(ADDR_SHT20) == ACK)   // device addr
	{ 
		if(I2C_WriteByte(command) == ACK)    // command
		 {
			do{
		   	  HAL_Delay(1);
			    I2C_Start(); 
			  } while(I2C_WriteByte(ADDR_SHT20 | 0x01) == NO_ACK);   // polling read status
			MSB = I2C_ReadByte(ACK);  // read data
			LSB = I2C_ReadByte(ACK); 
			I2C_ReadByte(NO_ACK);     // read check sum
			I2C_Stop(); 
			LSB &= 0xfc; 
			temp = MSB*256 + LSB; 
 
			if (command==(CMD_RH))   // read RH 
			  { Humidity = (temp*125)/65536 - 6; 
			    return Humidity; 
			  } 
			else 
			  { Temperature = (temp*175.72)/65536 - 46.85; 
			    return Temperature; 
			  }
		 }
	}
	return 0;  // error
}

// SetDataSHT20, same as ReadSHT20
// Input command = CMD_RH(0xF5) / CMD_TEM(0xF3)
// output: float / char* temperature or humidity 
//===========================================================================
void GetDataSHT20(uint8_t *buff, uint8_t cmd)  
//===========================================================================
{
  FloatChar TagTemp;
	
  TagTemp.x = ReadSHT20( cmd );    // read data from SHT20

  buff[0] = TagTemp.y[0];
  buff[1] = TagTemp.y[1];
  buff[2] = TagTemp.y[2];
  buff[3] = TagTemp.y[3];
}
