#ifndef DRIVERS_H
#define DRIVERS_H

#include "main.h"
#include <stdbool.h>
#include <math.h>
//#include <stdio.h>
//#include <stdlib.h>

#define DAC_WRITE_TO_REG 0x00
#define DAC_UPDATE_REG_N 0x01
#define DAC_WRITE_N_AND_UPDATE_ALL 0x02
#define DAC_WRITE_N_AND_UPDATE_N 0x03
#define DAC_POWER_DOWN_N 0x04
#define DAC_POWER_DOWN_ALL 0x05
#define DAC_SELECT_INT_REF 0x06
#define DAC_SELECT_EXT_REF 0x07

#define DAC_CHANNEL_A 0x00
#define DAC_CHANNEL_B 0x01
#define DAC_CHANNEL_C 0x02
#define DAC_CHANNEL_D 0x03
#define DAC_CHANNEL_ALL 0x0F

#define FIRMWARE_PAGE_OFFSET  0x3F800 
//TODO: check NVIC VectTab
#define NVIC_VectTab_FLASH				0x08000000
#define PARAMS_WORD_CNT 	sizeof(FlashMap) / sizeof(uint32_t)

//TODO: check formula
#define DAC_RANGE 40

typedef struct 
{
		double					coeff_a;
		double					coeff_b;
		double					coeff_c;

		uint8_t serial_number;
} FlashMapTypeDef;



void DAC_channel_init(void);
bool DAC_SPI_Transmit (uint8_t data, uint8_t command, uint8_t channel);

void FLASH_init(void);
void WriteStruct_to_FLASH (void);
void Flash_ReadParams(void);

uint8_t VoltInterpolation (double attenuation);

#endif
