/*********************************************************
FLASH Memory KEY, Optimization Level -1 
Author: jhshen
Date: 2021-01-17， for STM32F10x
注意：使用串口1进入，不同系列芯片串口1寄存器地址不同
**********************************************************/

#include "FLASH_KEY.h"
#include "md5.h"
//#include "tim.h"

//#include "string.h"
#include "stdio.h"
 
const uint32_t KEY0=0xffffffff, KEY1=0xffffffff, KEY2=0xffffffff, KEY3=0xffffffff;    // 空密钥
//const uint32_t KEY0=0xa04d0f7a, KEY1=0x95b453c3, KEY2=0x03183d32, KEY3=0xd4f1eac4;    // 特定板子0密钥
//const uint32_t KEY0=0xe34074d1, KEY1=0x01f81cd6, KEY2=0x80135085, KEY3=0x2d571316;    // 特定板子1密钥
//const uint32_t KEY0=0xadc6f8a5, KEY1=0xaa206242, KEY2=0xf9a25737, KEY3=0x5fa0b576;    // 特定板子2密钥

/*
//FLASH Erase Page and write one word
//===========================================================================
void WriteFlashWord(uint32_t addr, uint32_t data)
//===========================================================================
{
	// unLock FLASH
  HAL_FLASH_Unlock();
	
	//2 Erase FLASH
	// Init FLASH_EraseInitTypeDef
	FLASH_EraseInitTypeDef sf;
	sf.TypeErase = FLASH_TYPEERASE_PAGES;
	sf.PageAddress = addr;
	sf.NbPages = 1;
	// set PageError
	uint32_t PageError = 0;
	// Call erase function 
	HAL_FLASHEx_Erase(&sf, &PageError);  // Erase one Page!!

	// 3. write FLASH
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data); // write 32 bits word
	
	//4. Lock FLASH
  HAL_FLASH_Lock();
}
*/



/*  Authorized testing ....  */
// return 0 if author verify success
// Loopback TXD and RXD when first time running 
//===========================================================================
int AuthorVerify(void)
//===========================================================================
{	
 volatile unsigned int temp0=(unsigned int)&KEY0, temp1=(unsigned int)&KEY1, temp2=(unsigned int)&KEY2, temp3=(unsigned int)&KEY3;    // Address
 unsigned char rx_buf[]= "Hello";   // {'H', 'e', 'l', 'l', 'o', '\0'}; 
 unsigned char tx_buf[]= "XXXXX";  
 uint8_t  c1;			
 uint32_t ky[4];	

    uint8_t *msg;
    size_t len;
    int i;
    uint8_t result[16];   // 16 bytes
    uint32_t chip_id[4];

	  chip_id[0] = CHIP_ID0;
	  chip_id[1] = CHIP_ID1;
	  chip_id[2] = CHIP_ID2;
	  chip_id[3] = 0x3b6d7a59;   // private key

    msg = (uint8_t *) chip_id;
	  len = 16;  // 16 bytes
/*
    t0 = __HAL_TIM_GET_COUNTER(&htim6);
    for (i = 0; i < 100; i++)   // 实测每次MD5计算耗时140us
      {  md5((uint8_t*)msg, len, result);   }
    t1 = __HAL_TIM_GET_COUNTER(&htim6);
		printf ("\r\nMD5 100 = %d\r\n", t1-t0);
		for (i = 0; i < 16; i++)    printf("%2.2x", result[i]);
*/   

//    t0 = __HAL_TIM_GET_COUNTER(&htim6);
//    for (i = 0; i < 100; i++)
		 md5_hash((uint8_t*)msg, len, result);  // 实测每次MD5_hash计算耗时48us
//    t1 = __HAL_TIM_GET_COUNTER(&htim6);
//  	printf ("\r\nMD5_hash 100 = %d\r\n", t1-t0);
//    for (i = 0; i < 16; i++)   printf("%2.2x", result[i]);
 
//		printf("\r\n");
//    for (i = 0; i < 16; i++)  printf("%2.2x", result[i]);   // display result
		
		for (i=0; i<4; i++) {  // combine to 4 words
	     ky[i] = (uint32_t)result[i*4+0]<<24 | (uint32_t)result[i*4+1]<<16 | (uint32_t)result[i*4+2]<<8 | (uint32_t)result[i*4+3];
		 }
		
//    printf("\r\n");
//    for (i = 0; i < 4; i++)   printf("%08x", ky[i]);

		if ( (*(__IO uint32_t *)(temp0) == 0xFFFFFFFF) \
			&& (*(__IO uint32_t *)(temp1) == 0xFFFFFFFF) \
		  && (*(__IO uint32_t *)(temp2) == 0xFFFFFFFF) \
		  && (*(__IO uint32_t *)(temp2) == 0xFFFFFFFF) )    // empty Chip, set KEY
    	{
			  while ((USART1_ISR & RNE_bit) != 0)  c1 = USART1_RDR;  // read empty USART data reg.
			  c1++;
			 
			  HAL_UART_Transmit(&huart1, tx_buf, 1, 0);     // send 'X'
        if (HAL_UART_Receive(&huart1, rx_buf, 1, 200))  // receive 1 char, wait for 0.2s
		      {   // over time 
//			    printf ("UART over time. reStart please... \r\n");
//		      WriteFlashWord(0x08000000,0);   //  $$$$$$$$$$$$$$$$$$$$$ //
//			    while(1);
            return(1); 			// fail		
		      } 
				
			if (rx_buf[0] == 'X')	  // last char of Tx
		    {
   	      HAL_FLASH_Unlock();	
	  	    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, temp0, ky[0]);   // 32 bits word
		      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, temp1, ky[1]);   // 32 bits word
		      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, temp2, ky[2]);   // 32 bits word
		      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, temp3, ky[3]);   // 32 bits word
		      HAL_FLASH_Lock();
					
// 打印写入的密钥，用于复制记录
  		 printf("\r\nconst uint32_t KEY0=0x%08x, KEY1=0x%08x, KEY2=0x%08x, KEY3=0x%08x;\r\n", \
					  *(__IO uint32_t *)(temp0), *(__IO uint32_t *)(temp1),*(__IO uint32_t *)(temp2),*(__IO uint32_t *)(temp3) );
					
		    }
			 else {   // no 'X'
//			     printf ("Password error.  ReStart please.... \r\n");
// 		       WriteFlashWord(0x08000000,0);   //  $$$$$$$$$$$$$$$$$$$$$ //
//			     while(1);	 
				   return(2);  // fail
			   }
		 }	// set key

// 校验密钥：		 
    if ( ( *(__IO uint32_t *)(temp0) == (uint32_t)ky[0]) \
			&& ( *(__IO uint32_t *)(temp1) == (uint32_t)ky[1]) \
		  && ( *(__IO uint32_t *)(temp2) == (uint32_t)ky[2]) \
		  && ( *(__IO uint32_t *)(temp3) == (uint32_t)ky[3])   ) 
				{  
//				    printf(" Verify OK! \r\n");

// 打印特定板子密钥
//		    printf("\r\nconst uint32_t KEY0=0x%08x, KEY1=0x%08x, KEY2=0x%08x, KEY3=0x%08x;\r\n", \
					  *(__IO uint32_t *)(temp0), *(__IO uint32_t *)(temp1),*(__IO uint32_t *)(temp2),*(__IO uint32_t *)(temp3) );
					 return(0);  // sucess
				}	 
			else 
			  {
//					 printf(" Verify Fail! Boomming !!!\r\n");
//		       WriteFlashWord(0x08000000,0);   //  Flash first page boomming...$$$$$$$$$$$$$$$$$$$$$ //
					 return(3);  // fail
				}
}   

