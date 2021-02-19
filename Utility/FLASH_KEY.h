#ifndef  __FLASH_KEY_H__
#define  __FLASH_KEY_H__

#include "main.h"
#include "usart.h"

// STM32F103 USART reg mapaing:	
	
#define USART1_ISR  (*((volatile unsigned int *) (0x40013800 + 0x00) ))
#define USART1_RDR  (*((volatile unsigned int *) (0x40013800 + 0x04) ))	
#define USART1_TDR  (*((volatile unsigned int *) (0x40013800 + 0x04) ))	
/*
#define USART2_ISR  (*((volatile unsigned int *) (0x40004400 + 0x00) ))	
#define USART2_RDR  (*((volatile unsigned int *) (0x40004400 + 0x04) ))	
#define USART2_TDR  (*((volatile unsigned int *) (0x40004400 + 0x04) ))	

#define USART3_ISR  (*((volatile unsigned int *) (0x40004800 + 0x00) ))	
#define USART3_RDR  (*((volatile unsigned int *) (0x40004800 + 0x04) ))	
#define USART3_TDR  (*((volatile unsigned int *) (0x40004800 + 0x04) ))	
*/
/*
// STM32F3xx USART3 SFR: 不同系列芯片串口寄存器地址不同
#define USART3_ISR  (*((volatile unsigned int *) (0x40004800 + 0x1c) ))	
#define USART3_RDR  (*((volatile unsigned int *) (0x40004800 + 0x24) ))	
#define USART3_TDR  (*((volatile unsigned int *) (0x40004800 + 0x28) ))	
*/

#define TXE_bit 0x00000080
#define TC_bit  0x00000040
#define RNE_bit 0x00000020


// STM32F1xx Chip_ID address，不同系列芯片chip-id地址不同
#define CHIP_ID0  (*((volatile unsigned int *) (0x1FFFF7E8+0) ))
#define CHIP_ID1  (*((volatile unsigned int *) (0x1FFFF7E8+4) ))
#define CHIP_ID2  (*((volatile unsigned int *) (0x1FFFF7E8+8) ))

void WriteFlashWord(uint32_t addr, uint32_t data);
int AuthorVerify(void);

#endif
