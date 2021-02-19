#ifndef YEECOM_H
#define YEECOM_H
#include "main.h"
#include "string.h"
#include "cJSON.h"
#include "FIFO.h"
#define YComUart huart1	
extern UART_HandleTypeDef YComUart;						//��װ���Ӻ�DTU�����Ӵ���


HAL_StatusTypeDef EstablishMqttConnection(const char* ProductKey,const char* DeviceName,const char* DeviceSecret);

// Description: �ж�DTU�Ƿ񿪻���ɣ�ע��������
HAL_StatusTypeDef checkDtuStatu(void );
// Description: DTU mqtt������Ϣ����
HAL_StatusTypeDef initDtu(const char* ProductKey,const char* DeviceName,const char* DeviceSecret);
// Description: �ж�DTU�Ƿ������ϰ�����
HAL_StatusTypeDef checkConnection(void );

// Description: ����һ����Ϣ��������
HAL_StatusTypeDef SendMessageToAliIOT(uint16_t ip,uint16_t op,uint16_t iAngle,uint16_t oAngle,uint16_t tmp,uint8_t statu);

// Description: ����DTU������ATָ�������ATָ���OK��ERROR��
HAL_StatusTypeDef SendAtCommand(char* command);
// Description: ͨ��������DTU����ַ���
HAL_StatusTypeDef SendCharToDtu(char* msg);
// Description: ͨ��������DTU���u8����
HAL_StatusTypeDef SendU8ToDtu(uint8_t* msg,uint16_t len);

// Description: ��֯�ϱ�����Ϊ�涨��ʽ�ַ���
char* FormUploadData(uint16_t ip,uint16_t op,uint16_t iAngle,uint16_t oAngle,uint16_t tmp,uint8_t statu);
// Description: ���ɰ����ϱ����ݵİ�����Alink Json
char *FormJson(char* sendMsg);

// Description: �����·�����
void GetDownloadData(char* data,uint16_t* ip,uint16_t* op,uint16_t* iAngle,uint16_t* oAngle,uint16_t* tmp,uint16_t* statu);

#endif
