#ifndef __NTC_H
#define __NTC_H

#include "stm32f10x.h"

int StartNTC(uint16_t ad_res);
float ConverseToTemperature(int R);
//void NTCConverseToPackage();

//extern volatile int16_t NTCTemperature;  //原始温度值
//extern volatile int16_t NTCTemperature_Cal;  //校正后的温度值
extern uint32_t const NTC_103_INTERGER[161];
extern uint32_t const NTC_103_FRACTION[160];
#endif
