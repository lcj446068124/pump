/*
 * @Descripttion: 
 * @version: 
 * @Author: MichaelHu
 * @Date: 2020-06-18 18:14:14
 * @LastEditors: ChangjianLiu
 * @LastEditTime: 2021-02-18 16:12:50
 */
#ifndef __FIFO_H
#define __FIFO_H
#include "main.h"
#define FIFO_OK                  0
#define FIFO_ERROR_PARAM         1
#define FIFO_ERROR_MEM           2
#define FIFO_ERROR_FULL          3
#define FIFO_ERROR_EMPTY         4
#define FIFO_ERROR_BUSY          5

#define FIFO_BUF_LENGTH			300	
#define FIFO_DMA		        0  //采用interupt模式

typedef struct
{
    uint32_t size;                      //FIFO buffer size
    uint32_t front;                     //FIFO next read position
    uint32_t staraddr;                  //FIFO buffer start address
    uint32_t endaddr;                   //FFIFO buffer end address
		uint32_t CNDTR;					   					//empty space count
    uint8_t *buffer;               
} FIFOTYPE;

uint32_t FIFO_Init(FIFOTYPE **fifo, uint32_t fifosize, uint8_t dma_flag);
uint32_t FIFO_Clear(FIFOTYPE *fifo, uint8_t dma_flag);
uint32_t FIFO_Read(FIFOTYPE *fifo, uint8_t *data, uint8_t mode, uint8_t dma_flag);
uint32_t FIFO_ReadN(FIFOTYPE *fifo, uint8_t *data, uint16_t length, uint8_t dma_flag);
uint32_t FIFO_Status(FIFOTYPE *fifo, uint8_t dma_flag);
uint32_t FIFO_Remove(FIFOTYPE *fifo, uint32_t RemoveNum);
//////////////////////////////////////////////////////////////////////////////////////user code
// Description: 初始化ReceivedJsonBuffer缓存
void init_Json_Buffer(void);
// Description: 从FiFo中取出一条完整的Json并存入ReceivedJsonBuffer中
HAL_StatusTypeDef usart1_Process(void);

#endif
