#include "FIFO.h"


// **************************************************************
// Function: FIFO_Init
// Parameters: FIFO type pointer address, queue size，是否采用DMA方式
// Return: > = 0 initialization success
// Description: Initialize the FIFO queue
// **************************************************************

uint32_t FIFO_Init(FIFOTYPE **fifo, uint32_t fifosize, uint8_t dma_flag)
{
    volatile uint32_t da;
    if (fifo == NULL || fifosize == 0)
    {
        return FIFO_ERROR_PARAM;
    }

#ifdef configSUPPORT_DYNAMIC_ALLOCATION
    *fifo = pvPortMalloc(sizeof(FIFOTYPE));
#else
    *fifo = malloc(sizeof(FIFOTYPE));
#endif
    if ((*fifo) == NULL)
    {
        return FIFO_ERROR_MEM;
    }

#ifdef configSUPPORT_DYNAMIC_ALLOCATION
    (*fifo)->buffer = pvPortMalloc(fifosize);
#else
    (*fifo)->buffer = malloc(fifosize);
#endif
    if ((*fifo)->buffer == NULL)
    {
        return FIFO_ERROR_MEM;
    }

    (*fifo)->size = fifosize;
    (*fifo)->staraddr = (uint32_t)(&(*fifo)->buffer[0]);           //Record FIFO buffer start address
    (*fifo)->endaddr = (uint32_t)(&(*fifo)->buffer[fifosize - 1]); //Record FIFO buffer end address
    (*fifo)->front = (*fifo)->staraddr;                       //FIFO next read data address

    (*fifo)->CNDTR = fifosize;

    if (dma_flag)
    {
        //MYDMA_Config(DMA1_Channel5, (uint32_t)&USART1->DR, (uint32_t)(*fifo)->staraddr, (*fifo)->size);
				// do DMA config
    }

    return FIFO_OK;
}

// **************************************************************
// Function: FIFO_Clear
// Parameter: FIFO type pointer address，是否采用DMA方式
// Return: FIFO_OK
// Description: empty FIFO queue
// **************************************************************
uint32_t FIFO_Clear(FIFOTYPE *fifo, uint8_t dma_flag)
{
    volatile uint32_t da;
    if (fifo == NULL)
        return FIFO_ERROR_PARAM;
    fifo->front = fifo->staraddr; //Set the next read address to start with FIFO buffer

    if (dma_flag)
    {
       // MYDMA_Config(DMA1_Channel5, (uint32_t)&USART1->DR, (uint32_t)fifo->staraddr, fifo->size);
			 // DMA CLEAR
    }
    else
    {
        fifo->CNDTR = fifo->size;
    }

    return FIFO_OK;
}

// **************************************************************
// Function: FIFO_Remove
// Parameter: FIFO type pointer address，RemoveNum
// Return: FIFO_OK，FIFO_ERROR_PARAM
// Description: remove the FIFO queue part of the data
// **************************************************************
uint32_t FIFO_Remove(FIFOTYPE *fifo, uint32_t RemoveNum)
{
    if (fifo == NULL)
        return FIFO_ERROR_PARAM;

    for (int i = 0; i < RemoveNum; i++)
    {
        if (fifo->front == fifo->endaddr)
        {
            fifo->front = fifo->staraddr;
        }
        else
        {
            fifo->front++; //Set the next read address to start with FIFO buffer
        }
    }

    return FIFO_OK;
}

// **************************************************************
// Function: FIFO_Read
// Parameters: queue pointer, 1byte data pointer，读取模式（是否出队），是否采用DMA方式
// Return: > = 0 read successfully
// Description: read 1 byte data from the FIFO queue
// **************************************************************
uint32_t FIFO_Read(FIFOTYPE *fifo, uint8_t *data, uint8_t mode, uint8_t dma_flag)
{
    if (fifo == NULL)
        return FIFO_ERROR_PARAM;
    if (FIFO_Status(fifo, dma_flag) == 0)
    {
        return FIFO_ERROR_EMPTY;
    }
    *data = (uint8_t)(*((uint8_t *)(fifo->front)));
    if (fifo->front == fifo->endaddr)
    {
        if (mode)
            fifo->front = fifo->staraddr;
    }
    else
    {
        if (mode)
            fifo->front++;
    }
    return FIFO_OK;
}

// **************************************************************
// Function: FIFO_ReadN
// Parameters: queue pointer, 1byte data pointer, 读取长度，是否采用DMA方式
// Return: = 0 read successfully
// Description: read N byte data from the FIFO queue
// **************************************************************
uint32_t FIFO_ReadN(FIFOTYPE *fifo, uint8_t *data, uint16_t length, uint8_t dma_flag)
{
    if (fifo == NULL)
        return FIFO_ERROR_PARAM;
    if (FIFO_Status(fifo, dma_flag) < length)
    {
        return FIFO_ERROR_EMPTY;
    }

    for (int i = 0; i < length; i++)
    {
        *data++ = (uint8_t)(*((uint8_t *)(fifo->front)));
        if (fifo->front == fifo->endaddr)
        {
            fifo->front = fifo->staraddr;
        }
        else
        {
            fifo->front++; //Set the next read address to start with FIFO buffer
        }
    }
    return FIFO_OK;
}

// **************************************************************
// Function: FIFO_Status
// Parameters: queue pointer,dma_flag
// Return: > 0 queue has not read data
// Description: Get the number of not read data
// **************************************************************
uint32_t FIFO_Status(FIFOTYPE *fifo, uint8_t dma_flag)
{
    int res;
    int nextsave;
    if (dma_flag)
    {
        nextsave = (uint32_t)fifo->endaddr + 1 - (uint32_t)DMA1_Channel5->CNDTR;
    }
    else
    {
        nextsave = (uint32_t)fifo->endaddr + 1 - (uint32_t)fifo->CNDTR;
    }

    res = nextsave - (uint32_t)(fifo->front);
    if (res < 0)
    {
        res = ((uint32_t)(fifo->endaddr) + 1 - (uint32_t)(fifo->front)) + (nextsave - (uint32_t)fifo->staraddr);
    }
    return res;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// user code

#define JsonBufferSize 300
char ReceivedJsonBuffer[JsonBufferSize];				//下发Json缓存
uint8_t numOfUnpairedBrace = 0;									//未匹配括号数量，此处通过Json中的左右大括号数量匹配，相等时认为接收完成一条Json
uint16_t JsonBufferPtr = 0;
uint8_t continueRead = 0;												//是否继续接收
uint16_t time2_cnt = 0;													//定时器2 counter，一秒加1，判断接收是否超时使用
uint16_t RetryTime = 31;													//尝试初始化DTU时间间隔counter
extern FIFOTYPE *uartFIFO;
extern TIM_HandleTypeDef htim2;

// **************************************************************
// Function: init_Json_Buffer
// Parameters: None
// Return:None
// Description: 初始化接收Json缓存
// **************************************************************

void init_Json_Buffer(){
	memset(ReceivedJsonBuffer,0,sizeof(ReceivedJsonBuffer));
  continueRead = 0;
	numOfUnpairedBrace = 0;
	JsonBufferPtr = 0;
}


// **************************************************************
// Function: HAL_TIM_PeriodElapsedCallback
// Parameters: TIM pointer
// Return:None
// Description: 判断一条下发Json接收是否超时（3s）
// **************************************************************
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim2.Instance)
	{
		RetryTime++;
		//printf("%d\r\n",RetryTime);
		if(RetryTime > 600){
			RetryTime = 31;
		}
		time2_cnt++;
    if (continueRead) 
    {
       if (time2_cnt >= 3)//从收到首位开始3秒还没有收全，放弃接收，清空已接收缓存
       {
						init_Json_Buffer();
             //printf("ERROR DATA FIFO LEN = %d\r\n", FIFO_Status(uartFIFO));
       }
    }
	}
}
// **************************************************************
// Function: usart1_Process
// Parameters: None
// Return:HAL_ERROR，HAL_OK
// Description: 从FiFo中取出一条完整的Json并存入ReceivedJsonBuffer中
// **************************************************************
HAL_StatusTypeDef usart1_Process()
{
    uint8_t read_Data;
		if(continueRead){																						//如果上一条消息没有收完
			while(numOfUnpairedBrace != 0 && FIFO_Status(uartFIFO, 0) > 0){
					if(JsonBufferPtr >= JsonBufferSize - 1){
						init_Json_Buffer();
						return HAL_ERROR;
					}
					read_Data = 0;
					FIFO_Read(uartFIFO, &read_Data, 1, 0);
					if(read_Data == '{')
						numOfUnpairedBrace++;
					else if(read_Data == '}')
						numOfUnpairedBrace--;
					ReceivedJsonBuffer[JsonBufferPtr++] = read_Data;
				}
			if(numOfUnpairedBrace == 0){
					ReceivedJsonBuffer[JsonBufferPtr++] = '\0';
					continueRead = 0;
					return HAL_OK;
				}else{
					continueRead = 1;
					return HAL_ERROR;
				}
		}else{
			if (FIFO_Status(uartFIFO, 0) >= 100){												//最小消息体大小大于100字节
				read_Data = 0;
				while(FIFO_Status(uartFIFO, 0) > 0 && read_Data != '{'){
					FIFO_Read(uartFIFO, &read_Data, 1, 0);
				}
				if(read_Data != '{')
					return HAL_ERROR;
				init_Json_Buffer();
				ReceivedJsonBuffer[JsonBufferPtr++] = '{';
				numOfUnpairedBrace++;
				while(numOfUnpairedBrace != 0 && FIFO_Status(uartFIFO, 0) > 0){//判断是否接收完成或者读取到末尾
					if(JsonBufferPtr >= JsonBufferSize - 1){
						init_Json_Buffer();
						return HAL_ERROR;
					}
					read_Data = 0;
					FIFO_Read(uartFIFO, &read_Data, 1, 0);
					if(read_Data == '{')
						numOfUnpairedBrace++;
					else if(read_Data == '}')
						numOfUnpairedBrace--;
					ReceivedJsonBuffer[JsonBufferPtr++] = read_Data;
				}
				if(numOfUnpairedBrace == 0){																		//接收完成
					ReceivedJsonBuffer[JsonBufferPtr++] = '\0';
					return HAL_OK;
				}else{																													//读取到末尾
					continueRead = 1;
					time2_cnt = 0;
					return HAL_ERROR;
				}
			}
			return HAL_ERROR;
		}
}

