#ifndef AT24C02_H
#define AT24C02_H

//#include "stm32f10x.h"
#include "main.h"

#define ADDR_AT24C02  0xA2      // I2C device address
#define Zero_addr  0x10         // Zero point value set address
#define Full_addr (Zero_addr+3) // Full scale value set address

uint8_t AT24C02_write_data(uint8_t data_addr, uint8_t data);
uint8_t AT24C02_read_data(uint8_t *Rdata, uint8_t data_addr);

#endif
