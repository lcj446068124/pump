#include "FIFO.h"


// **************************************************************
// Function: FIFO_Init
// Parameters: FIFO type pointer address, queue size���Ƿ����DMA��ʽ
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
// Parameter: FIFO type pointer address���Ƿ����DMA��ʽ
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
// Parameter: FIFO type pointer address��RemoveNum
// Return: FIFO_OK��FIFO_ERROR_PARAM
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
// Parameters: queue pointer, 1byte data pointer����ȡģʽ���Ƿ���ӣ����Ƿ����DMA��ʽ
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
// Parameters: queue pointer, 1byte data pointer, ��ȡ���ȣ��Ƿ����DMA��ʽ
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
char ReceivedJsonBuffer[JsonBufferSize];				//�·�Json����
uint8_t numOfUnpairedBrace = 0;									//δƥ�������������˴�ͨ��Json�е����Ҵ���������ƥ�䣬���ʱ��Ϊ�������һ��Json
uint16_t JsonBufferPtr = 0;
uint8_t continueRead = 0;												//�Ƿ��������
uint16_t time2_cnt = 0;													//��ʱ��2 counter��һ���1���жϽ����Ƿ�ʱʹ��
uint16_t RetryTime = 31;													//���Գ�ʼ��DTUʱ����counter
extern FIFOTYPE *uartFIFO;
extern TIM_HandleTypeDef htim2;

// **************************************************************
// Function: init_Json_Buffer
// Parameters: None
// Return:None
// Description: ��ʼ������Json����
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
// Description: �ж�һ���·�Json�����Ƿ�ʱ��3s��
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
       if (time2_cnt >= 3)//���յ���λ��ʼ3�뻹û����ȫ���������գ�����ѽ��ջ���
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
// Return:HAL_ERROR��HAL_OK
// Description: ��FiFo��ȡ��һ��������Json������ReceivedJsonBuffer��
// **************************************************************
HAL_StatusTypeDef usart1_Process()
{
    uint8_t read_Data;
		if(continueRead){																						//�����һ����Ϣû������
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
			if (FIFO_Status(uartFIFO, 0) >= 100){												//��С��Ϣ���С����100�ֽ�
				read_Data = 0;
				while(FIFO_Status(uartFIFO, 0) > 0 && read_Data != '{'){
					FIFO_Read(uartFIFO, &read_Data, 1, 0);
				}
				if(read_Data != '{')
					return HAL_ERROR;
				init_Json_Buffer();
				ReceivedJsonBuffer[JsonBufferPtr++] = '{';
				numOfUnpairedBrace++;
				while(numOfUnpairedBrace != 0 && FIFO_Status(uartFIFO, 0) > 0){//�ж��Ƿ������ɻ��߶�ȡ��ĩβ
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
				if(numOfUnpairedBrace == 0){																		//�������
					ReceivedJsonBuffer[JsonBufferPtr++] = '\0';
					return HAL_OK;
				}else{																													//��ȡ��ĩβ
					continueRead = 1;
					time2_cnt = 0;
					return HAL_ERROR;
				}
			}
			return HAL_ERROR;
		}
}

