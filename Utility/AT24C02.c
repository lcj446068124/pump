/*********************************************************
GPIO read/write EEPROM AT24C02/24C04, chip A0 is unuseful for AT24C04 
Author: jhshen@ecnu.edu.cn
Date: 20200603
**********************************************************/

#include "main.h"
#include "I2C_HAL.h"
#include "AT24C02.h"

// Write data to EEPROM, blocking 5ms, 
// return 0 if success; return 1 if no ACK 
//=============================================================
uint8_t AT24C02_write_data(uint8_t data_addr, uint8_t data)
//=============================================================
{
   I2C_Start(); 
	 if (I2C_WriteByte(ADDR_AT24C02) == ACK)   // Device Address
	  {	 
			if (I2C_WriteByte(data_addr)== ACK)   // Word Address
			   {	 
					 if (I2C_WriteByte(data)== ACK)   // data 
					 {
						 I2C_Stop();
						 HAL_Delay(5);  // writing time
						 return (0);   // write data success, return 0
					 }
				 }
	  }
   return(1);  // error, return 1
}


// Read data from EEPROM , return 1 if error
//=============================================================
uint8_t  AT24C02_read_data(uint8_t *Rdata, uint8_t data_addr)
//=============================================================
{
	I2C_Start();
	if (I2C_WriteByte(ADDR_AT24C02)== ACK)  // Device Address
	 {
		 if (I2C_WriteByte(data_addr)== ACK)  // Word Address
		  {
	      I2C_Start();          
	      if (I2C_WriteByte((ADDR_AT24C02 | 0x01))== ACK)  // write addr with Read Flag = 1
				 {
	         *Rdata = I2C_ReadByte(NO_ACK);    // read one byte, NO ACK 
           I2C_Stop();
					 return 0;  // success 
				 }
			 }
	 }
  return 1;	  // error
}
 

