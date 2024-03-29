#ifndef YEECOM_H
#define YEECOM_H
#include "main.h"
#include "string.h"
#include "cJSON.h"
#include "FIFO.h"

extern UART_HandleTypeDef huart1;						//封装板子和DTU的连接串口
extern UART_HandleTypeDef huart3;

// Description: 建立同Ali云的MQTT连接
HAL_StatusTypeDef EstablishMqttConnection(const char* ProductKey,const char* DeviceName,const char* DeviceSecret);

// Description: 判断DTU是否开机完成，注册上网络
HAL_StatusTypeDef checkDtuStatu(void );
// Description: DTU mqtt连接信息配置
HAL_StatusTypeDef initDtu(const char* ProductKey,const char* DeviceName,const char* DeviceSecret);
// Description: 判断DTU是否连接上阿里云
HAL_StatusTypeDef checkConnection(void );

// Description: 发送一条消息给阿里云
HAL_StatusTypeDef SendMessageToAliIOT(uint16_t Start,uint16_t Mode_Auto,uint32_t InWaterVal,uint32_t OutWaterVal,uint32_t PosWaterPd,uint32_t NegGasPd,uint32_t InWaterFd,uint32_t OutWaterFd,uint32_t PosMin,uint32_t PosMax);

// Description: 发送DTU配置类AT指令（配置类AT指令返回OK，ERROR）
HAL_StatusTypeDef SendAtCommand(char* command);
// Description: 通过串口向DTU输出字符串
HAL_StatusTypeDef SendCharToDtu(char* msg);
// Description: 通过串口向DTU输出u8数组
HAL_StatusTypeDef SendU8ToDtu(uint8_t* msg,uint16_t len);

// Description: 组织上报数据为规定格式字符串
char* FormUploadData(uint16_t Start,uint16_t Mode_Auto,uint32_t InWaterVal,uint32_t OutWaterVal,uint32_t PosWaterPd,uint32_t NegGasPd,uint32_t InWaterFd,uint32_t OutWaterFd,uint32_t PosMin,uint32_t PosMax);
// Description: 生成包含上报数据的阿里云Alink Json
char *FormJson(char* sendMsg);


#endif
