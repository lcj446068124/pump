#include "yeeCom.h"
extern FIFOTYPE *uartFIFO;
char uploadId[10] = "0";																	//��������ģ���������string 0~4294967295				
char propertyPostTopic[] = "thing.event.property.post";		//��������ģ���������
char AlinkVersion[] = "1.0";															//��������ģ���������
char PdataBuffer[300];																		//ƴ���ϱ����ݻ�����

uint8_t aRx_uart_DTU;																				//���ڽ�������
// **************************************************************
// Function: HAL_UART_RxCpltCallback
// Parameters: UART pointer
// Return:None
// Description: ���ڽ��ջص�����
// **************************************************************
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == YComUartInstance)//GPRS
	{
		 
		HAL_UART_Receive_IT(&YComUart, &aRx_uart_DTU, 1);					// enable INT receive next byte
		//HAL_UART_Transmit_IT(&PCUart,&aRx_uart_DTU, 1); 					//echo
		uartFIFO->buffer[FIFO_BUF_LENGTH - uartFIFO->CNDTR] = aRx_uart_DTU; //��ȡ���յ�������
    uartFIFO->CNDTR--;
    if (uartFIFO->CNDTR == 0)
    {
        uartFIFO->CNDTR = FIFO_BUF_LENGTH;
    }
	}
	
	if(huart->Instance == PCUartInstance)//pc														
	{

	}

}

// **************************************************************
// Function: initDtu
// Parameters: ��������Ԫ��ProductKey��DeviceName��DeviceSecret
// Return:HAL_OK
// Description: DTU mqtt������Ϣ����
// **************************************************************
HAL_StatusTypeDef initDtu(const char* ProductKey,const char* DeviceName,const char* DeviceSecret){
		const uint8_t tmpAreaSize = 200;
		char* tmpArea =(char*) malloc(tmpAreaSize);
		
		SendAtCommand("AT*WKMODE=10#");
		SendAtCommand("AT*GPRSMODE=1#");		
		SendAtCommand("AT*DTUID=IMEI#");
		SendAtCommand("AT*PHNO=#");
		SendAtCommand("AT*HBTIME=60#");		
		SendAtCommand("AT*DFI=100#");
		SendAtCommand("AT*DATAMD=0#");
		SendAtCommand("AT*DATAID=0#");		
		SendAtCommand("AT*APN=cmnet,,#");
		SendAtCommand("AT*SERVER1=0,iot-cn-nif1yb6ig1h.mqtt.iothub.aliyuncs.com,1883#");
		SendAtCommand("AT*SERVER2=0,,#");
	
		
	
		//SendAtCommand("AT*MQALI=g34tzHdyy7Z,P0000001,1513a690b37f87f7e18e5e9e149107dd");
		memset(tmpArea,0,tmpAreaSize);
		sprintf(tmpArea,"AT*MQALI=%s,%s,%s",ProductKey,DeviceName,DeviceSecret);
		SendAtCommand(tmpArea);
		
		//SendAtCommand("AT*MQTOP=/sys/g34tzHdyy7Z/P0000001/thing/event/property/post_reply&/sys/g34tzHdyy7Z/P0000001/thing/service/property/set,/sys/g34tzHdyy7Z/P0000001/thing/event/property/post#");
		memset(tmpArea,0,tmpAreaSize);
		sprintf(tmpArea,"AT*MQTOP=/sys/%s/%s/thing/event/property/post_reply&/sys/%s/%s/thing/service/property/set,/sys/%s/%s/thing/event/property/post#",ProductKey,DeviceName,ProductKey,DeviceName,ProductKey,DeviceName);
		SendAtCommand(tmpArea);
		
		
		SendAtCommand("AT*CHMODE=0#");
		SendAtCommand("AT*DBGMODE=0#");	
		
		free(tmpArea);
		tmpArea = NULL;
		
		return HAL_OK;

}

// **************************************************************
// Function: SendAtCommand
// Parameters: command point
// Return:HAL_OK
// Description: ����DTU������ATָ�������ATָ���OK��ERROR��
// **************************************************************

HAL_StatusTypeDef SendAtCommand(char* command){
	const uint8_t Response_len = 10;
	char DtuResponse[Response_len];
	printf("%s\r\n",command);
	while(1){
		memset(DtuResponse,0,sizeof(DtuResponse));
		FIFO_Clear(uartFIFO,0);
		HAL_UART_Transmit(&YComUart, (uint8_t *)command, (uint16_t)strlen(command), 500);
		HAL_Delay(1000);
		uint32_t len = FIFO_Status(uartFIFO, 0);
		if(len <= 0 || len > Response_len - 1)
			continue;
		FIFO_ReadN(uartFIFO, (uint8_t *)DtuResponse, len, 0);
		if(strncmp(DtuResponse,"OK",2) == 0){
			return HAL_OK;
		}
	}
}

// **************************************************************
// Function: EstablishMqttConnection
// Parameters:  const char* ProductKey,const char* DeviceName,const char* DeviceSecret
// Return:HAL_OK��HAL_ERROR
// Description: ����ͬAli�Ƶ�MQTT����
// **************************************************************
HAL_StatusTypeDef EstablishMqttConnection(const char* ProductKey,const char* DeviceName,const char* DeviceSecret){
		
		static uint8_t SuccessInit = 0;
			
		if(SuccessInit){
			return HAL_OK;
		}
	
		//����MCQTT����
		if(checkDtuStatu() == HAL_ERROR){																	//���DTU�Ƿ�ע��������
			SuccessInit = 0;
			return HAL_ERROR;
		}
		if(initDtu(ProductKey,DeviceName,DeviceSecret) == HAL_ERROR){			//����DTU mqtt������Ϣ
			SuccessInit = 0;
			return HAL_ERROR;
		}
		if(checkConnection() == HAL_ERROR){																//�ж�DTU�Ƿ������ϰ�����
			SuccessInit = 0;
			return HAL_ERROR;
		}	
		
		SuccessInit = 1;
		FIFO_Clear(uartFIFO,0);																//���DTU���ڽ���FIFO
		HAL_Delay(1000);	
		
		#ifdef debug
		printf("===================\r\n");
		printf("Init connection success\r\n");
		printf("===================\r\n");
		#endif
		
		return HAL_OK;
}



// **************************************************************
// Function: checkDtuStatu
// Parameters:  None
// Return:HAL_OK��5��δע������������HAL_ERROR
// Description: �ж�DTU�Ƿ񿪻���ɣ�ע��������
// **************************************************************
HAL_StatusTypeDef checkDtuStatu(){
	const uint8_t Response_len = 50;
	char DtuResponse[Response_len];
	uint8_t i = 0;
	for(i = 0;i < 5;i++){
		memset(DtuResponse,0,sizeof(DtuResponse));
		FIFO_Clear(uartFIFO,0);
		HAL_UART_Transmit(&YComUart, (uint8_t *)"AT*REG?", (uint16_t)strlen("AT*REG?"), 500);
		HAL_Delay(1000);
		uint32_t len = FIFO_Status(uartFIFO, 0);
		
		if(len <= 0 || len > Response_len - 1)
			continue;
		
		FIFO_ReadN(uartFIFO, (uint8_t *)DtuResponse, len, 0);
		if(strncmp(DtuResponse,"+REG: 1",7) == 0 || strncmp(DtuResponse,"+REG: 5",7) == 0){
			return HAL_OK;
		}
	}
	return HAL_ERROR;
}
// **************************************************************
// Function: checkDtuStatu
// Parameters:  None
// Return:HAL_OK��5��δ������aliyun��HAL_ERROR
// Description: �ж�DTU�Ƿ������ϰ�����
// **************************************************************
HAL_StatusTypeDef checkConnection(){
	const uint8_t Response_len = 50;
	char DtuResponse[Response_len];
	uint8_t i = 0;
	for(i = 0;i < 5;i++){
		memset(DtuResponse,0,sizeof(DtuResponse));
		FIFO_Clear(uartFIFO,0);
		HAL_UART_Transmit(&YComUart, (uint8_t *)"AT*GSTATE?", (uint16_t)strlen("AT*GSTATE?"), 500);
		HAL_Delay(1000);
		uint32_t len = FIFO_Status(uartFIFO, 0);
		
		if(len <= 0 || len > Response_len - 1)
			continue;
		
		FIFO_ReadN(uartFIFO, (uint8_t *)DtuResponse, len, 0);
		if(strncmp(DtuResponse,"+GSTATE:1",strlen("+GSTATE:1")) == 0){
			return HAL_OK;
		}
	}
	return HAL_ERROR;
}
// **************************************************************
// Function: SendMessageToAliIOT
// Parameters:  ����ѹ������ˮѹ������ˮ���Ƕȣ���ˮ���Ƕȣ������¶ȣ��豸����״̬
/*Start
Mode_Auto

��ˮ����ѹ
��ˮ����ѹ
��ˮѹ��
��ո�ѹ
��ˮ���Ƕ�
��ˮ���Ƕ�
*/
// Return:HAL_OK
// Description: ����һ����Ϣ��������
// **************************************************************
HAL_StatusTypeDef SendMessageToAliIOT(uint16_t Start,uint16_t Mode_Auto,uint32_t InWaterVal,uint32_t OutWaterVal,uint32_t PosWaterPd,uint32_t NegGasPd,uint32_t InWaterFd,uint32_t OutWaterFd){
	
	char* upLoadData = FormUploadData( Start, Mode_Auto, InWaterVal, OutWaterVal, PosWaterPd, NegGasPd, InWaterFd, OutWaterFd);
	char *JsonString = FormJson(upLoadData);
	printf("Sending: %s\r\n",JsonString);
	SendCharToDtu(JsonString);
	free(JsonString);
	printf("Done!\r\n");
	return HAL_OK;
}
// **************************************************************
// Function: SendCharToDtu
// Parameters:  message pointer
// Return:HAL_OK
// Description: ͨ��������DTU����ַ���
// **************************************************************
HAL_StatusTypeDef SendCharToDtu(char* msg){
	
	return HAL_UART_Transmit(&YComUart, (uint8_t *)msg, (uint16_t)strlen(msg), 500);
	
}
// **************************************************************
// Function: SendU8ToDtu
// Parameters:  message pointer��message len
// Return:HAL_OK
// Description: ͨ��������DTU���u8����
// **************************************************************
HAL_StatusTypeDef SendU8ToDtu(uint8_t* msg,uint16_t len){
	
	return HAL_UART_Transmit(&YComUart, (uint8_t *)msg, len, 500);
	
}

// **************************************************************
// Function: FormUploadData
// Parameters: ����ѹ������ˮѹ������ˮ���Ƕȣ���ˮ���Ƕȣ������¶ȣ��豸����״̬
// Return:HAL_OK
// Description: ��֯�ϱ�����Ϊ�涨��ʽ�ַ���
// **************************************************************
char* FormUploadData(uint16_t Start,uint16_t Mode_Auto,uint32_t InWaterVal,uint32_t OutWaterVal,uint32_t PosWaterPd,uint32_t NegGasPd,uint32_t InWaterFd,uint32_t OutWaterFd){
			memset(PdataBuffer,0,sizeof(PdataBuffer));
			sprintf(PdataBuffer,"start,%u,mode_auto,%u,inwaterval,%u,outwaterval,%u,poswaterpd,%u,neggaspd,%u,inwaterfd,%u,outwaterfd,%u",Start, Mode_Auto, InWaterVal, OutWaterVal, PosWaterPd, NegGasPd, InWaterFd, OutWaterFd);
			return PdataBuffer;
}


// **************************************************************
// Function: FormJson
// Parameters: �ϱ�����ָ��
// Return:��װ��İ�����Alink Jsonָ��
// Description: ���ɰ����ϱ����ݵİ�����Alink Json
// **************************************************************
char * FormJson(char* sendMsg){
		char *JsonString  = NULL;
		cJSON *AlinkMessage = NULL;
		cJSON * id = NULL;
		cJSON * method = NULL;
		cJSON * version = NULL;
		cJSON *params = NULL;
		cJSON * Pdata = NULL;
	
		AlinkMessage = cJSON_CreateObject();
		if(AlinkMessage == NULL)
				goto end;
		
		id = cJSON_CreateStringReference(uploadId);
		if(id == NULL)
				goto end;
		cJSON_AddItemToObject(AlinkMessage,"id",id);
		
		method = cJSON_CreateStringReference(propertyPostTopic);
		if(method == NULL)
				goto end;
		cJSON_AddItemToObject(AlinkMessage,"method",method);
		
		version = cJSON_CreateStringReference(AlinkVersion);
		if(version == NULL)
				goto end;
		cJSON_AddItemToObject(AlinkMessage,"version",version);
		
		params = cJSON_CreateObject();
		if(params == NULL)
				goto end;
		cJSON_AddItemToObject(AlinkMessage,"params",params);
		
		Pdata = cJSON_CreateStringReference(sendMsg);
		if(Pdata == NULL)
				goto end;
		cJSON_AddItemToObject(params,"DATA",Pdata);
	
		JsonString =  cJSON_PrintUnformatted(AlinkMessage); 

		end:		
		cJSON_Delete(AlinkMessage);
		return JsonString;
}









