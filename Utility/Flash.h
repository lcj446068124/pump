#ifndef  __FLASH_H__
#define  __FLASH_H__

#include "main.h"

#define FLASH_PARA_ADDR   0x0803F800     // last 2KB of STM32F103RC/VC 256KB (0x08000000~ 0x0803FFFF)

#define FLASH_COUNT_ADDR  0x0803F000     // 2KB of STM32F103RC/VC 256KB (0x08000000~ 0x0803FFFF)


void ReadFlashNWord(uint32_t Offset, int *ReadBuf, uint32_t Numb);
void WriteFlashNWord(uint32_t Offset, int *Buf, uint32_t Numb);

#endif
