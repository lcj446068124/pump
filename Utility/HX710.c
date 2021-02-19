#include "main.h"

#define HX710_SCK_H   HAL_GPIO_WritePin(HX710_CK_GPIO_Port, HX710_CK_Pin, GPIO_PIN_SET)
#define HX710_SCK_L   HAL_GPIO_WritePin(HX710_CK_GPIO_Port, HX710_CK_Pin, GPIO_PIN_RESET)
#define HX710_SDA    HAL_GPIO_ReadPin(HX710_DT_GPIO_Port, HX710_DT_Pin)

// update int32 test value
//================================================================================
int Read_HX710(void)
//================================================================================
{  uint8_t  i;
   uint32_t DAT_HX710;

    HX710_SCK_L;   // HX710 CLK=0
    HX710_SCK_L;   // HX710 CLK=0
		DAT_HX710 =0;
    while(HX710_SDA);  // wait for DOUT=0;
	
    for (i=0;i<24;i++)
    {
 		   HX710_SCK_H;  // HX710 CLK=1
       DAT_HX710 = DAT_HX710<<1;
   		 HX710_SCK_L;  // HX710 CLK=0
			 if(HX710_SDA !=0 ) DAT_HX710++;
    }
    HX710_SCK_H;  // HX710 CLK=1, 25th clock
    __NOP();    __NOP();    __NOP();    __NOP();
		HX710_SCK_L;  // HX710 CLK=0

    if(DAT_HX710 & 0x800000)    // <0?
        DAT_HX710 |= 0xFF800000;
		
		return(DAT_HX710);  // return signed value
}  // Read_HX710
