#include "main.h"
#include <stdbool.h>
#include "drivers.h"
#include <math.h>

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern UART_HandleTypeDef huart1;

extern FLASH_EraseInitTypeDef hflash;
FlashMapTypeDef FlashMap;

uint8_t VoltInterpolation (double attenuation)
{
		double temp = 0.0f;
		double voltage = 0.0f;
		uint32_t test;

		FlashMap.coeff_a = 158.1;
		FlashMap.coeff_b = 0.005657;
  FlashMap.coeff_c = -156.3;

		attenuation = attenuation/2;
		temp = pow(attenuation, FlashMap.coeff_b);
		test = (uint32_t)temp;
		voltage =  FlashMap.coeff_a * temp + FlashMap.coeff_c;
  test = (uint32_t)voltage;
		test = (voltage * DAC_RANGE);
		return (test);
}

bool DAC_SPI_Transmit (uint8_t data, uint8_t command, uint8_t channel)
{
		uint8_t data_tx[3];
		data_tx[0] = (0xFF & ((command << 4 | 0x0F) & (channel | 0xF0)));
		data_tx[1] = data;
		data_tx[2] = 0x00;

		HAL_GPIO_WritePin(SPI2_MASTER_CS_GPIO_Port, SPI2_MASTER_CS_Pin, GPIO_PIN_RESET);
		if (HAL_SPI_Transmit(&hspi2, data_tx, sizeof(uint8_t) * 3, 1) == HAL_OK)
		{
				HAL_GPIO_WritePin(SPI2_MASTER_CS_GPIO_Port, SPI2_MASTER_CS_Pin, GPIO_PIN_SET);
				return (true);
		}
		else
		{
				return (false);
		}
}

void DAC_channel_init(void)
{
		DAC_SPI_Transmit( 0xFF, DAC_WRITE_N_AND_UPDATE_ALL, DAC_CHANNEL_C);
}

void FLASH_init(void)
{
		//LL_FLASH_SetLatency(FLASH_LATENCY_0);

		hflash.TypeErase = FLASH_TYPEERASE_PAGES;
		//hflash.PageAddress = pageAdr;
		hflash.NbPages = 1;

  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
		//FLASH_HalfCycleAccessCmd();
}

void WriteToFlash(uint32_t value)
{
		uint32_t pageError = 0;
		uint32_t pageAdr = NVIC_VectTab_FLASH | FIRMWARE_PAGE_OFFSET;

  hflash.PageAddress = pageAdr;


		HAL_FLASH_Unlock();
		HAL_FLASHEx_Erase(&hflash, &pageError);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, pageAdr, value);
  HAL_FLASH_Lock();
}

void WriteStruct_to_FLASH (void)
{
		uint32_t		pageError = 0;
  uint32_t		*source_adr = (void *)&FlashMap;
		uint32_t		pageAdr = NVIC_VectTab_FLASH + FIRMWARE_PAGE_OFFSET;

		hflash.PageAddress = pageAdr;
	
		HAL_FLASH_Unlock();
		HAL_FLASHEx_Erase(&hflash, &pageError);

  for (uint8_t i = 0; i < PARAMS_WORD_CNT; ++i) 
		{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(pageAdr + i*4), *(source_adr + i));
  }

		HAL_FLASH_Lock();
}

void Flash_ReadParams(void) 
{
	uint32_t *source_adr = (uint32_t *)(NVIC_VectTab_FLASH + FIRMWARE_PAGE_OFFSET);
	uint32_t *dest_adr = (void *)&FlashMap;                                          

	for (uint16_t i=0; i < PARAMS_WORD_CNT; ++i) 
	{                               
		*(dest_adr + i) = *(__IO uint32_t*)(source_adr + i);                  
	}
}

bool UART_CheckEndString (uint8_t *data)
{
		uint8_t i = 0;
		while ((data[i] != '\n'))
		{
				i++;
				if (i == 255)
						return (false);
		}
		return  (true);
}

