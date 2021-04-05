#ifndef YEECOM_H
#define YEECOM_H
#include "main.h"
#include "string.h"
#include "cJSON.h"
#include "FIFO.h"

extern UART_HandleTypeDef huart1;						//��װ���Ӻ�DTU�����Ӵ���
extern UART_HandleTypeDef huart3;

// Description: ����ͬAli�Ƶ�MQTT����
HAL_StatusTypeDef EstablishMqttConnection(const char* ProductKey,const char* DeviceName,const char* DeviceSecret);

// Description: �ж�DTU�Ƿ񿪻���ɣ�ע��������
HAL_StatusTypeDef checkDtuStatu(void );
// Description: DTU mqtt������Ϣ����
HAL_StatusTypeDef initDtu(const char* ProductKey,const char* DeviceName,const char* DeviceSecret);
// Description: �ж�DTU�Ƿ������ϰ�����
HAL_StatusTypeDef checkConnection(void );

// Description: ����һ����Ϣ��������
HAL_StatusTypeDef SendMessageToAliIOT(uint16_t Start,uint16_t Mode_Auto,uint32_t InWaterVal,uint32_t OutWaterVal,uint32_t PosWaterPd,uint32_t NegGasPd,uint32_t InWaterFd,uint32_t OutWaterFd);

// Description: ����DTU������ATָ�������ATָ���OK��ERROR��
HAL_StatusTypeDef SendAtCommand(char* command);
// Description: ͨ��������DTU����ַ���
HAL_StatusTypeDef SendCharToDtu(char* msg);
// Description: ͨ��������DTU���u8����
HAL_StatusTypeDef SendU8ToDtu(uint8_t* msg,uint16_t len);

// Description: ��֯�ϱ�����Ϊ�涨��ʽ�ַ���
char* FormUploadData(uint16_t Start,uint16_t Mode_Auto,uint32_t InWaterVal,uint32_t OutWaterVal,uint32_t PosWaterPd,uint32_t NegGasPd,uint32_t InWaterFd,uint32_t OutWaterFd);
// Description: ���ɰ����ϱ����ݵİ�����Alink Json
char *FormJson(char* sendMsg);


#endif
