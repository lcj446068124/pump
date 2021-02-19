//==============================================================================
//   TP07 STM32F103RCT6 Board. EmLab @ECNU China. 
//==============================================================================
// Project   :  SHT2x Sample Code (V1.0)
// File      :  I2C_HAL.c
// Author    :  jhshen@cs.ecnu.edu.cn
// Controller:  STM32F103RCT6
// Compiler  :  KEIL MDK V5
// Brief     :  I2C Hardware abstraction layer operation function
//==============================================================================

//---------- Includes ----------------------------------------------------------
#include "main.h"
#include "I2C_HAL.h"

void delay_us(uint16_t dt)     // delay 1us
{
	uint16_t i;
  for(i=0; i<(dt*8); i++);    // *8 ---> 1.1us @ 72MHz  
}

//==============================================================================
void I2C_Init (void)
//==============================================================================
{ uint8_t i=0;
	
  IIC_SDA_H;
  IIC_SCL_H;
	HAL_Delay(1);
	for(i=0; i<15; i++) {
	   IIC_SCL_L;
	   delay_us(5);
	   IIC_SCL_H;
     delay_us(5);
	  }
 	 HAL_Delay(50);
   IIC_SCL_L;
	 delay_us(10);	
	 IIC_SDA_L;    // Stop
   delay_us(10);
   IIC_SCL_H;
   delay_us(10);
   IIC_SDA_H;
   delay_us(200);
 
}

//==============================================================================
void I2C_Start (void)
//==============================================================================
{
    IIC_SDA_H;
    IIC_SCL_H;
    delay_us(10);
    IIC_SDA_L;
    delay_us(10);
	  IIC_SCL_L;
	  delay_us(10);
}

//==============================================================================
void I2C_Stop (void)
//==============================================================================
{
	 delay_us(20);
   IIC_SCL_L;
	 delay_us(1); 
   IIC_SDA_L; 
   delay_us(1);
   IIC_SCL_H;
   delay_us(10);
   IIC_SDA_H;
   delay_us(10);
}

// return ACK status: 0 = ACK; 1= no ACK
//==============================================================================
I2cAck IIC_Wait_Ack(void)
//==============================================================================
{
    uint8_t flag_ack=0;
	  IIC_SCL_L;
    IIC_SDA_H;
    delay_us(5);
	
    IIC_SCL_H;
    delay_us(5);
    while(READ_SDA && flag_ack<10)   // wait for ACK
    {
       flag_ack++;
       delay_us(1);
    }
    if(flag_ack>=10)   
      {
       I2C_Stop();
       return (NO_ACK);     // No ACK
      }
    IIC_SCL_L;
    return (ACK);    // ACK
}


//==============================================================================
void IIC_Ack(void)   // Generate ACK
//==============================================================================
{
   IIC_SCL_L;
   IIC_SDA_L;     // SDA=0
   delay_us(5);
   IIC_SCL_H;
   delay_us(5);
   IIC_SCL_L;
	
}

//==============================================================================
void IIC_NAck(void)  // No ACK
//==============================================================================
{
    IIC_SCL_L;
    IIC_SDA_H;    // SDA=1
    delay_us(5);
    IIC_SCL_H;
    delay_us(5); 
    IIC_SCL_L;
	
}

//==============================================================================
uint8_t I2C_WriteByte (uint8_t txd)  // return ACK status: 0 = ACK;  1 = no ACK
//==============================================================================
{
    uint8_t t=0, error=0;
	  IIC_SCL_L;
    for(t = 0; t < 8; t++)
    {     
     if(txd & 0x80)
	        IIC_SDA_H;
       else
	        IIC_SDA_L;      
     delay_us(5);   
     IIC_SCL_H;
     delay_us(5);
		 IIC_SCL_L;    	
     delay_us(3);
     txd <<= 1;   
    }	 
  IIC_SDA_H;       // release SDA-line
  IIC_SCL_H;       // clk #9 for ack
  delay_us(5);     // data set-up time (t_SU;DAT)
  if( READ_SDA == NO_ACK) error=1;   //check ack from i2c slave
  IIC_SCL_L;
  delay_us(30);           //wait time to see byte package on scope
  return error;           //return error code
}

//==============================================================================
uint8_t I2C_ReadByte (I2cAck ack)   // read 8 bit and send ACK / no ACK
//==============================================================================
{
   uint8_t receive = 0;
   uint8_t i=8; 
   IIC_SDA_H;                           //release SDA-line	
   IIC_SCL_L;   
//   delay_us(5);  
   for(i = 0; i < 8; i++ )
   {
       IIC_SCL_H;
       delay_us(5);  
       receive<<=1;
       if(READ_SDA)  receive |= 0x01; 
		   IIC_SCL_L; 
       delay_us(3); 
    }
	 
	 if (ack == ACK) IIC_Ack();
		  else IIC_NAck();
   return receive;
}

uint8_t I2C_CheckDev(uint8_t dev_addr)
{   
    I2C_Start();	
    if (I2C_WriteByte( dev_addr ) == ACK)
    {
         /* sensor is online*/
			 I2C_Stop();  
       return 1;
    }
    else
    {
          /* fail: send stop */
            I2C_Stop();
            return 0;
    }
}

