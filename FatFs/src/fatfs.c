/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "fatfs.h"
#include "TimeBase.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
//FIL SDFile;       /* File object for SD */

uint8_t retFlash;    /* Return value for Flash */
char FlashPath[4];   /* Flash logical drive path */
FATFS FlashFatFS;    /* File system object for Flash logical drive */
//FIL FlashFile;       /* File object for Flash */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */    

void MX_FATFS_Init(void) 
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */     
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
	RTC_TimeStruct RTCTime = Get_RTC_Time();
	return ( (RTCTime.Year - 1980) << 25 ) 	|
				 ( RTCTime.Month << 21 ) 	|
				 ( RTCTime.Date << 16 ) 	|
				 ( RTCTime.Hours << 11 ) 	|
				 ( RTCTime.Minutes << 5 ) 	|
				 ( RTCTime.Seconds << 0 );
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
