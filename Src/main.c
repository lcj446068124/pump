/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "I2C_HAL.h"
#include "SHT2x.h"
#include "SHT3x.h"
#include "Flash.h"
#include "yeeCom.h"
#include "FIFO.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern int Read_HX710(void);
extern int StartNTC(uint16_t ad_res);

void UART_ISR(void);
void TIM6_ISR(void);
void TIM7_ISR(void);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

#define  MAIN_MOTOR_OFF  HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET)  // OT1: 380v 三相固态继电器控制主电机
#define  MAIN_MOTOR_ON   HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET)

#define  GAS_OFF   HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_RESET)        // OT2: 进气阀门，常开，关机时ON 15s
#define  GAS_ON    HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET) 

#define  GATE24V_OFF   HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_RESET)    // OT3: 24V 阀门电源继电器
#define  GATE24V_ON    HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET)      // 

#define  BUZZ_OFF  HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET)        // OT6: 外接蜂鸣器 
#define  BUZZ_ON   HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET)

#define  START    (HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin))   // DI1: 外部启动按钮
#define  STOP     (HAL_GPIO_ReadPin(IN2_GPIO_Port, IN2_Pin))   // DI2: 外部停止按钮

#define ADC_CHANNEL_CNT  10 	 // ADC channel Num
#define ADC_CHANNEL_FRE  16	   // Single channel data number for average
uint16_t adc1_val_buf[ADC_CHANNEL_CNT * ADC_CHANNEL_FRE];  // buffer for ADC DMA trans
uint32_t adc1_aver_val[ADC_CHANNEL_CNT];

uint16_t Start=0;  // 开关机命令
uint16_t Mstate=0;  // 主状态机

uint16_t DAC1_val=0, DAC2_val=0;
uint16_t IO1_val=0, IO2_val=0, IO3_val=0, IO4_val=0;

uint16_t cnt10ms, cnt1s;

volatile int16_t NTCTemperature;  // Temperature convert from ADC value
int32_t iTemp, iHumi;

#define NegMin  1638  // -0.04KP
#define NegMax  1936  // -0.02KP
#define PosMin  1862  // 0.5MP
#define PosMax  2234  // 0.6MP
#define ConDlt  500   // 阀门控制电压与实际反馈电压的最大差值，500-->0.5v

#define PARAMETER_SIZE 10   // 共10个参数，读写Flash用													 
struct // Parameter 系统参数结构体
{
	int InWaterVal;    // 进水阀控制电压0~1000，对应输出0~10V
	int OutWaterVal;   // 出水阀控制电压0~1000，对应输出0~10V
	int InWaterFd;     // 进水阀反馈电压0~1000，对应输出0~10V
	int OutWaterFd;    // 出水阀反馈电压0~1000，对应输出0~10V
	
	int NegGasPd;      // 负压，正常范围气压-0.02~ -0.04（1936~1638）, -0.06（1340）, -0.08（1042）
                     //	+/-0.1KP 4-20mA传感器对应数据，0气压为12mA@150欧=1.8V（2234）；
	int PosWaterPd;    // 出水压力，正常范围气压0.5~ 0.6（1862 ~ 2234）
	                   // 1MP 4-20mA传感器对应数据，0压力为4mA@150欧=0.6V（745）；
	int IO1Val;
	int IO2Val;
	int IO3Val;
	int IO4Val;
//	int param[4];
} Sys;         // 系统参数Sys

FIFOTYPE *uartFIFO;			//Dtu串口接收Fifo指针

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 重定向printf函数
int fputc(int ch,FILE *f)
{
    uint8_t temp[1]={ch};
    HAL_UART_Transmit(&huart3, temp, 1, 5);   // USART3，阻塞方式发送一个字符
	return 0;
}

void delay1us(void)
{
	  for(int i=0; i<8; i++);    //1us
}

void delay5us(void)
{
	  for(int i=0; i< 60; i++);    //4.17us
}

void TIM6_ISR(void)
{
	 cnt10ms++;
}

void TIM7_ISR(void)
{
	 cnt1s++;
}

//	STM32F103 USART reg mapaing:
/*
#define USART1_ISR  (*((volatile unsigned int *) (0x40013800 + 0x00) ))
#define USART1_RDR  (*((volatile unsigned int *) (0x40013800 + 0x04) ))	
#define USART1_TDR  (*((volatile unsigned int *) (0x40013800 + 0x04) ))	

#define USART2_ISR  (*((volatile unsigned int *) (0x40004400 + 0x00) ))	
#define USART2_RDR  (*((volatile unsigned int *) (0x40004400 + 0x04) ))	
#define USART2_TDR  (*((volatile unsigned int *) (0x40004400 + 0x04) ))	
*/
#define USART3_ISR  (*((volatile unsigned int *) (0x40004800 + 0x00) ))	
#define USART3_RDR  (*((volatile unsigned int *) (0x40004800 + 0x04) ))	
#define USART3_TDR  (*((volatile unsigned int *) (0x40004800 + 0x04) ))	

#define TXE_bit 0x00000080
#define TC_bit  0x00000040
#define RNE_bit 0x00000020
	
uint8_t aRx_uart, Rx_uart, Rx_state, RxFirst, RxSecond, RxThird;
uint8_t TxBuffer[5];
uint8_t  uart3cnt=0, uart3_Rxcnt=0;

void UART_ISR(void)
{
  // PC Communication COM3, 38400bps
 uint16_t set_val=0;
	
 uart3cnt++;  // for Debug only, how many times enter the UART ISR
 if ((USART3_ISR & RNE_bit) != 0)   // Rx INT
  {  
		uart3_Rxcnt++;  // for Debug only, how many times enter the UART Rx ISR
//  	Rx_uart= aRx_uart;  // if Rx INT, aRx_uart is read by above HAL_UART_IRQHandler(&huart1), including Error processing
	   Rx_uart = USART3_RDR;  // read USART data reg.

  		if (Rx_state ==0) {     // First byte
			    RxFirst = Rx_uart;
			    switch(RxFirst) {
				      case 'G' :	TxBuffer[0] = 'O';
						              HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 1); 
						              aRx_uart = '?';  // clear command byte
                          break;						
				      case 'B' :	Start=1;   // 开机
						              TxBuffer[0] = 'B';
						              HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 1); 
						              aRx_uart = '?';  // clear command byte
                          break;						
				      case 'P' :	Start=0;   // 关机
						              TxBuffer[0] = 'P';
						              HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 1); 
						              aRx_uart = '?';  // clear command byte
                          break;						
						
	            case 'T' :       // Read Temperature
                          TxBuffer[0] = NTCTemperature>>8; TxBuffer[1] = NTCTemperature;
							            HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 2); 
                          aRx_uart = '?';  // clear command byte							
                          break;
	            case 'H' :       // Read Humidity
                          TxBuffer[0] = iHumi>>8; TxBuffer[1] = iHumi;
							            HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 2); 
                          aRx_uart = '?';  // clear command byte							
                          break;
							
	            case '1' :       // Read VI1
							            TxBuffer[0] = adc1_aver_val[1]>>8; TxBuffer[1] = adc1_aver_val[1];
							            HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 2);
                          aRx_uart = '?';  // clear command byte							
                          break;
	            case '2' :       // Read VI2
							            TxBuffer[0] = adc1_aver_val[2]>>8; TxBuffer[1] = adc1_aver_val[2];
							            HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 2);
                          aRx_uart = '?';  // clear command byte							
                          break;
	            case '3' :       // Read VI3
							            TxBuffer[0] = adc1_aver_val[3]>>8; TxBuffer[1] = adc1_aver_val[3];
							            HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 2);
                          aRx_uart = '?';  // clear command byte							
                          break;
	            case '4' :       // Read VI4
							            TxBuffer[0] = adc1_aver_val[4]>>8; TxBuffer[1] = adc1_aver_val[4];
							            HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 2);
                          aRx_uart = '?';  // clear command byte							
                          break;
	            case '5' :       // Read VI5
							            TxBuffer[0] = adc1_aver_val[5]>>8; TxBuffer[1] = adc1_aver_val[5];
							            HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 2);
                          aRx_uart = '?';  // clear command byte							
                          break;
	            case '6' :       // Read VI6
							            TxBuffer[0] = adc1_aver_val[6]>>8; TxBuffer[1] = adc1_aver_val[6];
							            HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 2);
                          aRx_uart = '?';  // clear command byte							
                          break;
	            case '7' :       // Read VI7
							            TxBuffer[0] = adc1_aver_val[7]>>8; TxBuffer[1] = adc1_aver_val[7];
							            HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 2);
                          aRx_uart = '?';  // clear command byte							
                          break;
	            case '8' :       // Read VIN8
							            TxBuffer[0] = adc1_aver_val[8]>>8; TxBuffer[1] = adc1_aver_val[8];
							            HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 2);
                          aRx_uart = '?';  // clear command byte							
                          break;
	            case '9' :       // Read VIN9
							            TxBuffer[0] = adc1_aver_val[9]>>8; TxBuffer[1] = adc1_aver_val[9];
							            HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 2);
                          aRx_uart = '?';  // clear command byte							
                          break;
							
	            case 'A' :       // Read Input status
                          TxBuffer[0] = 0xff;     //  Read value  
							            if(HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin)==0)  TxBuffer[0] = TxBuffer[0] & 0xFE;   // IN1
							            if(HAL_GPIO_ReadPin(IN2_GPIO_Port, IN2_Pin)==0)  TxBuffer[0] = TxBuffer[0] & 0xFD;   // IN2
							            if(HAL_GPIO_ReadPin(IN3_GPIO_Port, IN3_Pin)==0)  TxBuffer[0] = TxBuffer[0] & 0xFB;   // IN3
							            if(HAL_GPIO_ReadPin(IN4_GPIO_Port, IN4_Pin)==0)  TxBuffer[0] = TxBuffer[0] & 0xF7;   // IN4
							            if(HAL_GPIO_ReadPin(IN5_GPIO_Port, IN5_Pin)==0)  TxBuffer[0] = TxBuffer[0] & 0xEF;   // IN5
							            if(HAL_GPIO_ReadPin(IN6_GPIO_Port, IN6_Pin)==0)  TxBuffer[0] = TxBuffer[0] & 0xDF;   // IN6
							            if(HAL_GPIO_ReadPin(IN7_GPIO_Port, IN7_Pin)==0)  TxBuffer[0] = TxBuffer[0] & 0xBF;   // IN7
							
							            HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 1);
                          aRx_uart = '?';  // clear command byte							
                          break;
							case 'S' :           // Set Output
                          Rx_state=1;
							            break;
	            case 'U' :       // Set DAC1
                          Rx_state=1;
							            break;
	            case 'V' :       // Set DAC2
                          Rx_state=1;
							            break;
	            case 'I' :       // Set IO1 
                          Rx_state=1;
							            break;
	            case 'J' :       // Set IO2 
                          Rx_state=1;
							            break;
	            case 'K' :       // Set IO3 
                          Rx_state=1;
							            break;
	            case 'L' :       // Set IO4 
                          Rx_state=1;
							
							default  :  Rx_state=0;
							            aRx_uart = '?';  // clear command byte			
		       								break;
 		        }
 			   }
	
 	  else if (Rx_state ==1) {     // Second byte
			    RxSecond = Rx_uart;
			    switch(RxFirst) {
					 case 'S' :	 if ((RxSecond & 0x01) !=0) 
						                   HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
					                else HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);
					             if ((RxSecond & 0x02) !=0) 
						                   HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
					                else HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_RESET);
					             if ((RxSecond & 0x04) !=0) 
						                   HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
					                else HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_RESET);
					             if ((RxSecond & 0x08) !=0) 
						                   HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_SET);
					                else HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_RESET);
					             if ((RxSecond & 0x10) !=0) 
						                   HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_SET);
					                else HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET);
					             if ((RxSecond & 0x20) !=0) 
						                   HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_SET);
					                else HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);
					             if ((RxSecond & 0x40) !=0) 
						                   HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
					                else HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
											 TxBuffer[0] = 'O';
											 Rx_state=0;
						           aRx_uart = '?';  // clear command byte
						           HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 1); 
                       break;
						 
           case 'U' :       
	         case 'V' :       
           case 'I' :       
           case 'J' :       
           case 'K' :       
           case 'L' :       
						 
                          Rx_state=2;
                          break;
           default  :    
						              Rx_state=0;
 						              aRx_uart = '?';  // clear command byte
                          break;
				 }
			 }
	 
   else if (Rx_state ==2) {     // Third byte
			    RxThird = Rx_uart;
			    switch(RxFirst) {
           case 'U' :       // Set DAC1 0~1000 --> 0~10.00v
						              set_val = RxSecond *256 + RxThird;
					                if (set_val>1000) TxBuffer[0] = 'E';
					                   else {
                      		      Sys.InWaterVal = set_val;
												 	      WriteFlashNWord(0, (int*)&Sys, PARAMETER_SIZE);		// 一次写入全部参数	
															  DAC1_val = (uint16_t)((float)Sys.InWaterVal * 4.095);
															  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC1_val );
													      TxBuffer[0] = 'O';
														 }
													Rx_state=0;
						              aRx_uart = '?';  // clear command byte
						              HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 1); 
                          break;
           case 'V' :       // Set DAC2 0~1000 --> 0~10.00v
						              set_val = RxSecond *256 + RxThird;
					                if (set_val>1000) TxBuffer[0] = 'E';
					                   else {
                      		      Sys.OutWaterVal = set_val;
	                              WriteFlashNWord(0, (int*)&Sys, PARAMETER_SIZE);		// 一次写入全部参数	
															  DAC1_val = (uint16_t)((float)Sys.OutWaterVal * 4.095);
															  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC1_val );
													      TxBuffer[0] = 'O';
														 }
													Rx_state=0;
						              aRx_uart = '?';  // clear command byte
						              HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 1); 
                          break;

           case 'I' :       // Set IO1
						              IO1_val = RxSecond *256 + RxThird;
					                if (IO1_val>2000) TxBuffer[0] = 'E';
					                   else {
                     		        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, IO1_val);  // 设置占空比 _val/1000
													      TxBuffer[0] = 'O';
														 }
													Rx_state=0;
						              aRx_uart = '?';  // clear command byte
						              HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 1); 
                          break;
           case 'J' :       // Set IO2
						              IO2_val = RxSecond *256 + RxThird;
					                if (IO2_val>2000) TxBuffer[0] = 'E';
					                   else {
                     		        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, IO2_val);  // 设置占空比 _val/1000
													      TxBuffer[0] = 'O';
														 }
													Rx_state=0;
						              aRx_uart = '?';  // clear command byte
						              HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 1); 
                          break;
           case 'K' :       // Set IO3
						              IO3_val = RxSecond *256 + RxThird;
					                if (IO3_val>2000) TxBuffer[0] = 'E';
					                   else {
                     		        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, IO3_val);  // 设置占空比 _val/1000
													      TxBuffer[0] = 'O';
														 }
													Rx_state=0;
						              aRx_uart = '?';  // clear command byte
						              HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 1); 
                          break;
           case 'L' :       // Set IO4
						              IO4_val = RxSecond *256 + RxThird;
					                if (IO4_val>2000) TxBuffer[0] = 'E';
					                   else {
                     		        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, IO4_val);  // 设置占空比 _val/1000
													      TxBuffer[0] = 'O';
														 }
													Rx_state=0;
						              aRx_uart = '?';  // clear command byte
						              HAL_UART_Transmit_IT(&huart3,(uint8_t*)TxBuffer, 1); 
                          break;
						
 		        }
		}
	 
  HAL_UART_Receive_IT(&huart3, &aRx_uart, 1);   // enable INT receive next byte
	}  // Rx INT

}




extern char ReceivedJsonBuffer[];																		//下发Json缓存
char receive_command[100];																					//抽取下发Json中的控制指令缓存
const char* productKey = "g34tzHdyy7Z";															//阿里云三元组productKey
const char* deviceName = "P0000001";																//阿里云三元组deviceName
const char* deviceSecret = "1513a690b37f87f7e18e5e9e149107dd";			//阿里云三元组deviceSecret
extern uint16_t RetryTime;																					//尝试初始化DTU时间间隔counter
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t TempErrorCnt=0;
	uint16_t i;
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_GPIO_Init();
	MX_DMA_Init();   // ***** must before ADC1_init();  ***** 
  MX_ADC1_Init();
	FIFO_Init(&uartFIFO, FIFO_BUF_LENGTH, 0);									//初始化Fifo长度为FIFO_BUF_LENGTH
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_SPI2_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();																										
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim6);   // TIM6  INT enable
	HAL_TIM_Base_Start_IT(&htim7);   // TIM7  INT enable
	HAL_TIM_Base_Start_IT(&htim2);   // TIM2  INT enable	8M input frequency 1s interupt
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);      // TIM8_CH1 PWM enable
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);      // TIM8_CH2 PWM enable
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);      // TIM8_CH3 PWM enable
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);      // TIM8_CH4 PWM enable

  
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);		// LED1 ON
	
  HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);  
  HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
	
	 
	
  	HAL_UART_Receive_IT(&huart3, &aRx_uart, 1);  // UART3 for PC communication
		HAL_UART_Receive_IT(&huart1, &aRx_uart, 1);  // UART1 for GPRS communication
	
		HAL_ADC_Start(&hadc1); 						
    HAL_ADCEx_Calibration_Start(&hadc1);    // ADC 自校正
		HAL_Delay(20);
		
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &adc1_val_buf, (ADC_CHANNEL_CNT*ADC_CHANNEL_FRE));
  	HAL_Delay(10);
//    HAL_ADC_Start_IT(&hadc1);   // ADC1 INT

    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

    ReadFlashNWord(0, (int*)&Sys, PARAMETER_SIZE);   // 读取系统参数
		if ((Sys.InWaterVal<100)||(Sys.InWaterVal>900)||(Sys.OutWaterVal<100)||(Sys.OutWaterVal>900))  // 1.0V~9.0V
		  {	
				Sys.InWaterVal = 520;   //  5.2v
				Sys.OutWaterVal = 380;  //  3.8v
				Sys.IO1Val = 0;          // 0mA
				Sys.IO1Val = 400;        // 4mA
				Sys.IO1Val = 1000;       // 10mA
				Sys.IO1Val = 2000;       // 20mA
/*
				Sys.param[0] = 100;
				Sys.param[1] = 200;
				Sys.param[2] = 300;
				Sys.param[3] = 400;
*/				
	      // write FLASH, Erase one Page!
	      WriteFlashNWord(0, (int*)&Sys, PARAMETER_SIZE);		// 一次写入全部参数	
			}
			
		// read parameters from Flash Memory
    ReadFlashNWord(0, (int*)&Sys, PARAMETER_SIZE);    // 一次读出全部参数

		
//		DAC1_val = Sys.InWaterVal *4;   // x4.096
//		DAC2_val = Sys.OutWaterVal*4;
//		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC1_val);
//		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC2_val);
			
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Sys.IO1Val);  // 设置占空比0%,  0mA
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, Sys.IO2Val);  // 设置占空比20%, 4mA
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Sys.IO3Val);  // 设置占空比50%, 10mA
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, Sys.IO4Val);  // 设置占空比100%, 20mA

		GATE24V_OFF;     // 关阀门电源
    MAIN_MOTOR_OFF;  // 关380v主电机
    GAS_OFF;         // 释放进气阀（常开）
//    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
//    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
		
	  HAL_Delay(20);
	  Mstate=0;
			
//    printf("\n\rBEGIN Pump......\n\r"); 
//			printf("InW : %d\r\n", Sys.InWaterVal);
//			printf("OutW: %d\r\n", Sys.OutWaterVal);


		//读取并输出当前MCU工作频率
		RCC_ClkInitTypeDef RCC_ClkInitStruct;
		uint32_t pFLatency;
		HAL_RCC_GetClockConfig(&RCC_ClkInitStruct,&pFLatency);
		printf("ClockType: %d\nSYSCLKSource %d\nAHBCLKDivider %d\nAPB1CLKDivider %d\nAPB2CLKDivider %d\n",RCC_ClkInitStruct.ClockType,RCC_ClkInitStruct.SYSCLKSource,RCC_ClkInitStruct.AHBCLKDivider,RCC_ClkInitStruct.APB1CLKDivider,RCC_ClkInitStruct.APB2CLKDivider);
		uint32_t sysclock = 0;   
		sysclock = HAL_RCC_GetSysClockFreq();
		printf("\r\nsysclock: %d\r\n",sysclock);
		
		
		//临时变量，存储进气压力，出水压力，进水阀角度，出水阀角度，环境温度，设备开机状态，测试使用
		uint16_t ip = 0;
		uint16_t op = 0;
		uint16_t ia = 0;
		uint16_t oa = 0;
		uint16_t tmp = 0;
		uint16_t sta = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint16_t cnt_time = 5 * 60 * 10;											//计算发送间隔，此处为10 min 
  while (1)
  {
			cnt_time++;
		
			if(RetryTime >= 30){																//距离上一次初始化30s
				if(EstablishMqttConnection(productKey,deviceName,deviceSecret) == HAL_ERROR){			//如果初始化失败
					#ifdef debug
					printf("===================\r\n");
					printf("Init connection failed\r\n");
					printf("===================\r\n");
					#endif
					RetryTime = 0;																		//设置定时器目标值为0，等待30s后再次尝试初始化
				}
			}
			
			if(usart1_Process() == HAL_OK){										//从FiFo中取出一条完整的Json并存入ReceivedJsonBuffer中
				memset(receive_command,0,sizeof(receive_command));
				printf("%s \r\n",ReceivedJsonBuffer);
				//解析Json数据
				cJSON * receiveJson = cJSON_Parse(ReceivedJsonBuffer);
				cJSON * myParams = cJSON_GetObjectItem(receiveJson,"params");
				cJSON *Pdata = cJSON_GetObjectItem(myParams,"DATA");
				if(Pdata!= NULL){
					//获取命令字符串
					strcpy(receive_command,Pdata->valuestring);
					printf("%s \r\n",receive_command);
					//解析命令字符串
					GetDownloadData(receive_command,&ip,&op,&ia,&oa,&tmp,&sta);
					//打印更新后的存储进气压力，出水压力，进水阀角度，出水阀角度，环境温度，设备开机状态
					printf("%u %u %u %u %u %u\r\n",ip,op,ia,oa,tmp,sta);
					//此次循环结束立刻上报更新信息
					cnt_time = 5 * 60 * 10;
				}
				cJSON_Delete(receiveJson);
			}
	
			if(cnt_time >= 5 * 60 * 10 ){								//到达10s，开始上报进气压力，出水压力，进水阀角度，出水阀角度，环境温度，设备开机状态
				SendMessageToAliIOT(ip,op,ia,oa,tmp,sta);	//上报进气压力，出水压力，进水阀角度，出水阀角度，环境温度，设备开机状态
				cnt_time = 0;															//清除计时
			}
			
		  switch (Mstate)
			{
				case 0 :  if (Start==1)    // 串口 或 外部启动
					            Mstate=1;
					        break;
				
				case 1 :  
	
					        GATE24V_ON;
          				DAC1_val = (float)Sys.InWaterVal *4.095;   // x4.095
		              DAC2_val = (float)Sys.OutWaterVal*4.095;
		              HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC1_val);
		              HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC2_val);
                  MAIN_MOTOR_ON;
				          cnt1s=0;
				          Mstate=2;
					        break;
        
				case 2 :  if (cnt1s>10)  Mstate=3;    // Delay 10s
				          break;
				
				case 3 :  
				          if ( (Sys.InWaterFd > (Sys.InWaterVal-ConDlt)) &&(Sys.InWaterFd < (Sys.InWaterVal+ConDlt)) \
										&& (Sys.OutWaterFd >(Sys.OutWaterVal-ConDlt))&&(Sys.OutWaterFd <(Sys.OutWaterVal+ConDlt)) )      // 阀门控制是否到位？
                    {
											if ((Sys.NegGasPd>NegMin)&&(Sys.NegGasPd<NegMax)&&(Sys.PosWaterPd>PosMin)&&(Sys.PosWaterPd<PosMax) )	// 压力是否正常？
											  { 
													Mstate=4;      // 阀门控制到位、气压正常 
												}
												else
												{
													Mstate=5;      // 气压不正常
												}	
									  } 
										else  Mstate=6;      // 阀门控制不到位
									break;	
										     
				        
				case 4 :  Mstate=3;
									break;
										
				case 5 :	Mstate=3;
									break;
        case 6 :  Mstate=3;
									break;
				
        case 10:  GAS_ON;					// 关机序列，关进气阀					
				          cnt1s=0;
				          Mstate=11;
				          break;
				
				case 11:  if (cnt1s < 15) break;  // 15s
				          GATE24V_OFF;     // 关阀门电源
//                  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
//		              HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
				          MAIN_MOTOR_OFF;  // 关380v主电机
				          GAS_OFF;         // 释放进气阀（常开）
									Mstate=0;
									Start=0;
									break;
				          
			}  // switch
		
			if  (START==0) Start=1;     // 外部启动
			if  (STOP==0)  Start=0;     // 外部停止
			if ( (Start==0) && (Mstate>0) && (Mstate<10)) Mstate=10;   // 关机
			
			
  		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);		// LED Flash
 
      for(i=0; i<ADC_CHANNEL_CNT; i++)    // clear ADC value
       {
         adc1_aver_val[i] = 0;
       }
    
		   /* sum for each channel from DMA buffer */
      for(i=0; i<ADC_CHANNEL_FRE; i++)   //
       {
         adc1_aver_val[0] +=  adc1_val_buf[i*ADC_CHANNEL_CNT+0];    // ADC1_IN10, NTC
         adc1_aver_val[1] +=  adc1_val_buf[i*ADC_CHANNEL_CNT+1];    // ADC1_IN11, VIN1(4~20mA)
         adc1_aver_val[2] +=  adc1_val_buf[i*ADC_CHANNEL_CNT+2];    // ADC1_IN12, VIN2(4~20mA)
         adc1_aver_val[3] +=  adc1_val_buf[i*ADC_CHANNEL_CNT+3];    // ADC1_IN13, VIN3(4~20mA)
         adc1_aver_val[4] +=  adc1_val_buf[i*ADC_CHANNEL_CNT+4];    // ADC1_IN0, VIN4(0~10V), 
         adc1_aver_val[5] +=  adc1_val_buf[i*ADC_CHANNEL_CNT+5];    // ADC1_IN1, VIN5(0~10V)
         adc1_aver_val[6] +=  adc1_val_buf[i*ADC_CHANNEL_CNT+6];    // ADC1_IN2, VIN6(0~5V)
         adc1_aver_val[7] +=  adc1_val_buf[i*ADC_CHANNEL_CNT+7];    // ADC1_IN3, VIN7(0~5V, 浑浊度)
         adc1_aver_val[8] +=  adc1_val_buf[i*ADC_CHANNEL_CNT+8];    // ADC1_IN6, VIN8(4~20mA)
         adc1_aver_val[9] +=  adc1_val_buf[i*ADC_CHANNEL_CNT+9];    // ADC1_IN7, VIN9(4~20mA)
       }
    
	  // 	average
      for(i=0; i<ADC_CHANNEL_CNT; i++)
       {
         adc1_aver_val[i] /= ADC_CHANNEL_FRE;
//				 printf("ADC RANK%d: %d\r\n",i, adc1_aver_val[i]);
       }
			
	   Sys.InWaterFd  = (adc1_aver_val[4]*1000)/4096;    // 进水阀反馈电压0~1000，对应输出0~10V
	   Sys.OutWaterFd = (adc1_aver_val[5]*1000)/4096;    // 出水阀反馈电压0~1000，对应输出0~10V
	   Sys.NegGasPd   = adc1_aver_val[2];    // 负气压，正常范围气压-0.02~ -0.04（1936~1638）, -0.06（1340）, -0.08（1042）
                                           //	+/-0.1KP 4-20mA传感器对应数据，0气压为12mA@150欧=1.8V（2234）；
	   Sys.PosWaterPd = adc1_aver_val[3];    // 出水压力，正常范围气压0.5~ 0.6（1862 ~ 2234）
	                                         // 1MP 4-20mA传感器对应数据，0压力为4mA@150欧=0.6V（745）；
			 
      NTCTemperature = StartNTC(adc1_aver_val[0]);  // cal NTC Temperature
//		  printf("NTC Temp = %d C \r\n", NTCTemperature);
/*	
	    if ( (SHT20_GetTemp( &iTemp )==0) && (SHT20_GetRH( &iHumi ) ==0) )
			   { TempErrorCnt=0; 
//			     printf("SHT20: Temp = %03d C  RH = %02d \r\n", (iTemp+5)/10, (iHumi+50)/100);
	       }
 			  else TempErrorCnt++;
				 
	    if ( SHT3X_ReadData(&iTemp, &iHumi) ==0 ) 
			   { TempErrorCnt=0; 
//	         printf("SHT30: Temp = %03d C   RH = %02d%% \r\n", (iTemp+5)/10, (iHumi+50)/100);
				 }
				else TempErrorCnt++;
				 
//			if (TempErrorCnt>10)   	printf("SHT2x/3x Sensor error! \r\n");
*/    				

//      printf("Mstate: %d \r\n", Mstate);		
      HAL_Delay(200);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;  // PC13 No Alarm output !! 2020/12/17 MXCube 不能配置为无输出
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
   
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x17;
  sTime.Minutes = 0x5;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_DECEMBER;
  DateToUpdate.Date = 0x16;
  DateToUpdate.Year = 0x20;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 800-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7200-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7200-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 2-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 2070;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|HX710_CK_Pin|OUT2_Pin|OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZ_Pin|SPI2_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT6_Pin|OUT5_Pin|OUT4_Pin|OUT3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, I2C_SCL_Pin|I2C_SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED1_Pin HX710_CK_Pin OUT2_Pin OUT1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|HX710_CK_Pin|OUT2_Pin|OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : HX710_DT_Pin */
  GPIO_InitStruct.Pin = HX710_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HX710_DT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZ_Pin SPI2_SS_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin|SPI2_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT6_Pin OUT5_Pin OUT4_Pin OUT3_Pin */
  GPIO_InitStruct.Pin = OUT6_Pin|OUT5_Pin|OUT4_Pin|OUT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IN7_Pin */
  GPIO_InitStruct.Pin = IN7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IN7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN6_Pin */
  GPIO_InitStruct.Pin = IN6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IN6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN5_Pin IN4_Pin IN3_Pin IN2_Pin
                           IN1_Pin */
  GPIO_InitStruct.Pin = IN5_Pin|IN4_Pin|IN3_Pin|IN2_Pin
                          |IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C_SCL_Pin I2C_SDA_Pin */
  GPIO_InitStruct.Pin = I2C_SCL_Pin|I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
