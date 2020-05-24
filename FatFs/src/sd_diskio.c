/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   SD Disk I/O driver
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
/* USER CODE END Header */

/* Note: code generation based on sd_diskio_dma_rtos_template.c v2.0.2 as FreeRTOS is enabled. */

/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection*/

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "bsp_driver_sd.h"

#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include <string.h>
#include "Basic.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define QUEUE_SIZE         (uint32_t) 10
#define READ_CPLT_MSG      (uint32_t) 1
#define WRITE_CPLT_MSG     (uint32_t) 2
/*
 * the following Timeout is useful to give the control back to the applications
 * in case of errors in either BSP_SD_ReadCpltCallback() or BSP_SD_WriteCpltCallback()
 * the value by default is as defined in the BSP platform driver otherwise 30 secs
 */
#define SD_TIMEOUT 30 * 1000

#define SD_DEFAULT_BLOCK_SIZE 512

/*
 * Depending on the use case, the SD card initialization could be done at the
 * application level: if it is the case define the flag below to disable
 * the BSP_SD_Init() call in the SD_Initialize() and add a call to 
 * BSP_SD_Init() elsewhere in the application.
 */
/* USER CODE BEGIN disableSDInit */
 #define DISABLE_SD_INIT 
/* USER CODE END disableSDInit */

/* 
 * when using cachable memory region, it may be needed to maintain the cache
 * validity. Enable the define below to activate a cache maintenance at each
 * read and write operation.
 * Notice: This is applicable only for cortex M7 based platform.
 */
/* USER CODE BEGIN enableSDDmaCacheMaintenance */
#ifdef DCACHE_SIZE
 #define ENABLE_SD_DMA_CACHE_MAINTENANCE  1
 Static_AXIDMABuf uint8_t scratch[BLOCKSIZE];
#else
	__attribute__ ((aligned (4))) uint8_t scratch[BLOCKSIZE];
#endif
/* USER CODE BEGIN enableSDDmaCacheMaintenance */

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

static osMessageQId SDQueueID;
/* Private function prototypes -----------------------------------------------*/
static DSTATUS SD_CheckStatus(BYTE lun);
DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);
DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
  DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT SD_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
#if  _USE_WRITE == 1
  SD_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  SD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */
/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/
static DSTATUS SD_CheckStatus(BYTE lun)
{
  Stat = STA_NOINIT;

  if(BSP_SD_GetCardState() == MSD_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_initialize(BYTE lun)
{
  Stat = STA_NOINIT;
  /*
   * check that the kernel has been started before continuing
   * as the osMessage API will fail otherwise
   */
  if(osKernelRunning())
  {
#if !defined(DISABLE_SD_INIT)

    if(BSP_SD_Init() == MSD_OK)
    {
      Stat = SD_CheckStatus(lun);
    }

#else
    Stat = SD_CheckStatus(lun);
#endif

    /*
     * if the SD is correctly initialized, create the operation queue
     */

    if (Stat != STA_NOINIT)
    {
      osMessageQDef(SD_Queue, QUEUE_SIZE, uint16_t);
      SDQueueID = osMessageCreate (osMessageQ(SD_Queue), NULL);
    }
  }
  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status(BYTE lun)
{
  return SD_CheckStatus(lun);
}

/* USER CODE BEGIN beforeReadSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeReadSection */
/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  osEvent event;
  uint32_t timer;
#ifdef DCACHE_SIZE
	if( isDMABuf( (uint32_t)buff ) && ( ((uint32_t)buff & 0x3) == 0 ) )
	{	//缓冲区为非cache区域且四字节对齐
#else
	if( ((uint32_t)buff & 0x3) == 0 )
	{	//缓冲区四字节对齐
#endif
		//直接dma送进缓冲区
		if(BSP_SD_ReadBlocks_DMA((uint32_t*)buff,
														 (uint32_t) (sector),
														 count) == MSD_OK)
		{
			/* wait for a message from the queue or a timeout */
			event = osMessageGet(SDQueueID, SD_TIMEOUT);

			if (event.status == osEventMessage)
			{
				if (event.value.v == READ_CPLT_MSG)
				{
					if( count > 1 )
						SDMMC_CmdStopTransfer(SDMMC1);
					timer = osKernelSysTick() + SD_TIMEOUT;
					/* block until SDIO IP is ready or a timeout occur */
					while(timer > osKernelSysTick())
					{
						if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
						{
							
							res = RES_OK;
							Stat &= ~STA_NOINIT;
							break;
						}
					}
				}
			}
		}	
	}
#ifdef DCACHE_SIZE
	else if( ((uint32_t)buff & 0x1f) == 0 )
	{	//缓冲区32字节对齐
		//直接dma送进缓冲区
		//将对应Cache数据清除
		SCB_InvalidateDCache_by_Addr((uint32_t*)buff, count*BLOCKSIZE);
		if(BSP_SD_ReadBlocks_DMA((uint32_t*)buff,
														 (uint32_t) (sector),
														 count) == MSD_OK)
		{
			/* wait for a message from the queue or a timeout */
			event = osMessageGet(SDQueueID, SD_TIMEOUT);

			if (event.status == osEventMessage)
			{
				if (event.value.v == READ_CPLT_MSG)
				{
					if( count > 1 )
						SDMMC_CmdStopTransfer(SDMMC1);
					timer = osKernelSysTick() + SD_TIMEOUT;
					/* block until SDIO IP is ready or a timeout occur */
					while(timer > osKernelSysTick())
					{
						if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
						{
							
							res = RES_OK;
							Stat &= ~STA_NOINIT;
							break;
						}
					}
				}
			}
		}
	}
#endif
	else
	{
		//缓冲区非32字节对齐或非4字节对齐
		//把数据用dma传送至内置缓冲区在复制过去
		int i;
		uint8_t ret;
		for (i = 0; i < count; i++) {
      ret = BSP_SD_ReadBlocks_DMA((uint32_t*)scratch, (uint32_t)sector++, 1);
      if (ret == MSD_OK) {
        /* wait for a message from the queue or a timeout */
        event = osMessageGet(SDQueueID, SD_TIMEOUT);

        if (event.status == osEventMessage) {
          if (event.value.v == READ_CPLT_MSG) {
//						//将对应Cache数据清除
//						SCB_InvalidateDCache_by_Addr((uint32_t*)scratch, BLOCKSIZE);

						//把数据从内置缓冲区复制到buf
						memcpy(buff, scratch, BLOCKSIZE);
						buff += BLOCKSIZE;
          }
        }
      }
      else
      {
        break;
      }
    }

    if ((i == count) && (ret == MSD_OK))
		{
			timer = osKernelSysTick() + SD_TIMEOUT;
			/* block until SDIO IP is ready or a timeout occur */
			while(timer > osKernelSysTick())
			{
				if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
				{
					res = RES_OK;
					Stat &= ~STA_NOINIT;
					break;
				}
			}
		}
	}
  return res;
}

/* USER CODE BEGIN beforeWriteSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeWriteSection */
/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  osEvent event;
  uint32_t timer;

#ifdef DCACHE_SIZE
	if( isDMABuf( (uint32_t)buff ) && ( ((uint32_t)buff & 0x3) == 0 ) )
	{	//缓冲区为非cache区域或非四字节对齐		
#else
	if( ((uint32_t)buff & 0x3) == 0 )
	{	//缓冲区四字节对齐
#endif
		//直接用dma发送
		if(BSP_SD_WriteBlocks_DMA((uint32_t*)buff,
															(uint32_t) (sector),
															count) == MSD_OK)
		{
			/* Get the message from the queue */
			event = osMessageGet(SDQueueID, SD_TIMEOUT);
			
			if (event.status == osEventMessage)
			{
				if (event.value.v == WRITE_CPLT_MSG)
				{
					if( count > 1 )
						SDMMC_CmdStopTransfer(SDMMC1);
					
					timer = osKernelSysTick() + SD_TIMEOUT;
					/* block until SDIO IP is ready or a timeout occur */
					
					while(timer > osKernelSysTick())
					{
						if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
						{
							res = RES_OK;
							Stat &= ~STA_NOINIT;
							break;
						}
					}
				}
			}
		}
	}
#ifdef DCACHE_SIZE
	else if( ((uint32_t)buff & 0x1f) == 0 )
	{	//缓冲区32字节对齐或非4字节对齐
		//直接用dma发送
		
		//把Cache内容写入内存
		SCB_CleanDCache_by_Addr((uint32_t*)buff, count*BLOCKSIZE);
		if(BSP_SD_WriteBlocks_DMA((uint32_t*)buff,
															(uint32_t) (sector),
															count) == MSD_OK)
		{
			/* Get the message from the queue */
			event = osMessageGet(SDQueueID, SD_TIMEOUT);

			if (event.status == osEventMessage)
			{
				if (event.value.v == WRITE_CPLT_MSG)
				{
					if( count > 1 )
						SDMMC_CmdStopTransfer(SDMMC1);
					timer = osKernelSysTick() + SD_TIMEOUT;
					/* block until SDIO IP is ready or a timeout occur */
					while(timer > osKernelSysTick())
					{
						if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
						{						
							res = RES_OK;
							Stat &= ~STA_NOINIT;
							break;
						}
					}
				}
			}
		}
	}
#endif
	else
	{	//缓冲区非32字节对齐
		//把数据先复制到内置缓冲区再发送	
		int i;
    uint8_t ret;
		
    for (i = 0; i < count; i++) {
			//数据从buf拷贝到scratch
			memcpy((void *)scratch, (void *)buff, BLOCKSIZE);
      ret = BSP_SD_WriteBlocks_DMA((uint32_t*)scratch, (uint32_t)sector++, 1);
      if (ret == MSD_OK) {
        /* wait for a message from the queue or a timeout */				
				event = osMessageGet(SDQueueID, SD_TIMEOUT);
				if (event.status == osEventMessage)
				{
					if (event.value.v == WRITE_CPLT_MSG)
					{
						timer = osKernelSysTick() + SD_TIMEOUT;
						buff += BLOCKSIZE;											
						
						/* block until SDIO IP is ready or a timeout occur */
						while(timer > osKernelSysTick())
						{
							if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
							{								
								ret = RES_OK;
								break;
							}
						}
					}
				}              
      }
      else
      {
				res = MSD_ERROR;
        break;
      }
    }

    if ((i == count) && (ret == MSD_OK))
		{
			res = RES_OK;
			Stat &= ~STA_NOINIT;
		}
	}
  return res;
}
#endif /* _USE_WRITE == 1 */

/* USER CODE BEGIN beforeIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeIoctlSection */
/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  BSP_SD_CardInfo CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE BEGIN afterIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END afterIoctlSection */

/* USER CODE BEGIN callbackSection */ 
/* can be used to modify / following code or add new code */
/* USER CODE END callbackSection */
/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */

 /*
   ===============================================================================
    Select the correct function signature depending on your platform.
    please refer to the file "stm32xxxx_eval_sd.h" to verify the correct function
    prototype
   ===============================================================================
  */
//void BSP_SD_WriteCpltCallback(uint32_t SdCard)
void BSP_SD_WriteCpltCallback(void)
{
  /*
   * No need to add an "osKernelRunning()" check here, as the SD_initialize()
   * is always called before any SD_Read()/SD_Write() call
   */
  osMessagePut(SDQueueID, WRITE_CPLT_MSG, osWaitForever);
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */

  /*
   ===============================================================================
    Select the correct function signature depending on your platform.
    please refer to the file "stm32xxxx_eval_sd.h" to verify the correct function
    prototype
   ===============================================================================
  */
//void BSP_SD_ReadCpltCallback(uint32_t SdCard)
void BSP_SD_ReadCpltCallback(void)
{
  /*
   * No need to add an "osKernelRunning()" check here, as the SD_initialize()
   * is always called before any SD_Read()/SD_Write() call
   */
  osMessagePut(SDQueueID, READ_CPLT_MSG, osWaitForever);
}

/* USER CODE BEGIN lastSection */ 
/* can be used to modify / undefine previous code or add new code */
/* USER CODE END lastSection */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

