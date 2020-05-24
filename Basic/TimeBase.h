#pragma once

#include "stm32h743xx.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
	extern "C" {
#endif

#ifdef SYSFREQ480
	#define SYSCLK 480000000
	#define HCLK 24000000
	#define APB1CLK 120000000
	#define APB1TIMERCLK 240000000
	#define APB2CLK 120000000
	#define APB2TIMERCLK 240000000
	#define APB3CLK 120000000
	#define APB4CLK 120000000
#else
	#define SYSCLK 400000000
	#define HCLK 20000000
	#define APB1CLK 100000000
	#define APB1TIMERCLK 200000000
	#define APB2CLK 100000000
	#define APB2TIMERCLK 200000000
	#define APB3CLK 100000000
	#define APB4CLK 100000000
#endif
#define USART234578CLK 100000000
#define USART16CLK 100000000


#define TIMEBASECLK 10000000
		
	/*
		操作系统延时函数（通过挂起线程）
		只能在操作系统任务中使用
		t:秒单位延时时间
	*/
	inline void os_delay(double t)
	{
		TickType_t ticks = t*configTICK_RATE_HZ;
		if( ticks < 1 )
			ticks = 1;
		vTaskDelay(ticks);
	}
	
	/*RTC时间获取*/
		typedef struct
		{
			uint16_t Year;
			uint8_t Month;
			uint8_t Date;
			uint8_t WeekDay;

			uint8_t Hours;
			uint8_t Minutes;
			uint8_t Seconds;
			uint32_t SubSeconds;
			uint8_t TimeFormat;
			
		}RTC_TimeStruct;
	
		//获取RTC时间
		RTC_TimeStruct Get_RTC_Time();
		/*  设置RTC日期和时间
		 *  UTC_Time* T
		*/
		void Set_RTC_Time(const RTC_TimeStruct* T);
	/*RTC时间获取*/
	
#ifdef __cplusplus
	}
#endif