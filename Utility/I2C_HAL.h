#ifndef I2C_HAL_H
#define I2C_HAL_H
//==============================================================================
//   TP07 Board. EmLab @ECNU, China 
//==============================================================================
// Project   :  SHT2x Sample Code (V1.0)
// File      :  I2C_HAL.h
// Author    :  jhshen@cs.ecnu.edu.cn
// Controller:  STM32F103RCT6
// Compiler  :  KEIL MDK V5
// Brief     :  I2C Hardware abstraction layer head file
//==============================================================================

//---------- Includes ----------------------------------------------------------
#include "main.h"

//---------- Defines -----------------------------------------------------------
//I2C ports GPIO OD output
//The communication on SDA and SCL is done by switching pad direction
//For a low level on SCL or SDA, direction is set to output. For a high level on
//SCL or SDA, OD output (extern pull up resistor active)

#define IIC_SCL_H   HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET)
#define IIC_SCL_L   HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET)
#define IIC_SDA_H   HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_SET)
#define IIC_SDA_L   HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_RESET)
#define READ_SDA    HAL_GPIO_ReadPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin)


//---------- Enumerations ------------------------------------------------------
//  I2C level
typedef enum{
  LOW                      = 0,
  HIGH                     = 1,
}I2cLevel;

// I2C acknowledge
typedef enum{
  ACK                      = 0,
  NO_ACK                   = 1,
}I2cAck;

//==============================================================================
void I2C_Init (void);
//==============================================================================
//Initializes the ports for I2C interface

//==============================================================================
void I2C_Start (void);
//==============================================================================
// writes a start condition on I2C-bus
// input : -
// output: -
// return: -
// note  : timing (delay) may have to be changed for different microcontroller
//       _____
// SDA:       |_____
//       _______
// SCL :        |___

//==============================================================================
void I2C_Stop (void);
//==============================================================================
// writes a stop condition on I2C-bus
// input : -
// output: -
// return: -
// note  : timing (delay) may have to be changed for different microcontroller
//              _____
// SDA:   _____|
//            _______
// SCL :  ___|

//===========================================================================
uint8_t I2C_WriteByte (uint8_t txByte);
//===========================================================================
// writes a byte to I2C-bus and checks acknowledge
// input:  txByte  transmit byte
// output: -
// return: error
// note: timing (delay) may have to be changed for different microcontroller

//===========================================================================
uint8_t I2C_ReadByte (I2cAck ack);
//===========================================================================
// reads a byte on I2C-bus
// input:  rxByte  receive byte
// output: rxByte
// note: timing (delay) may have to be changed for different microcontroller

uint8_t I2C_CheckDev(uint8_t );

void delay_us(uint16_t );

#endif

