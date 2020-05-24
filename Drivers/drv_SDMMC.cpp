#include "drv_SDMMC.hpp"
#include "Basic.hpp"
#include "fatfs.h"

#include "semphr.h"
#include "stream_buffer.h"

SD_HandleTypeDef hsd1;
MDMA_HandleTypeDef hmdma_mdma_channel40_sdmmc1_end_data_0;
/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 4;
}

void init_drv_SDMMC()
{
	//¿ªÆôSD¿¨¼ì²âÒý½Å(PA15)
	RCC->AHB4ENR |= (1<<0);
	set_register( GPIOA->MODER , 0b00 , 30 , 2 );
	GPIOA->OTYPER |= (1<<15);
	set_register( GPIOA->PUPDR , 1 , 30 , 2 );

	MX_SDMMC1_SD_Init();
	retSD = FATFS_LinkDriver(&SD_Driver, SDPath);
}

extern "C" void SDMMC1_IRQHandler(void)
{
  HAL_SD_IRQHandler(&hsd1);
}
