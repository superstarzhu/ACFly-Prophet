#include "drv_RCPPM.hpp"
#include "Basic.hpp"
#include "TimeBase.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "ReceiverBackend.hpp"
#include "AC_Math.hpp"

static float RC_channels_value[8];
static uint8_t channels_count = 0;
extern "C" void TIM1_IRQHandlerTCB(void *pvParameter1, uint32_t ulParameter2 )
{
	ReceiverUpdate( "PPM", true, RC_channels_value, channels_count,0.01 );
	NVIC_EnableIRQ(TIM1_CC_IRQn);
}
static inline unsigned short count_short(unsigned short a,unsigned short b)
{
	if(b>a)return b-a;
	else return (unsigned short)65535-a+b+1;
}
extern "C" void TIM1_CC_IRQHandler()
{ 
	static uint16_t old_RC_cnt = 0;
	static uint8_t current_RC_channel = 255;
	
	#define RC_channel_valid ( current_RC_channel != 255 )
	#define set_RC_channel_invalid ( current_RC_channel = 255 )
	#define reset_RC_channel ( current_RC_channel = 0 )
	#define is_Synchro_Blank( channel ) ( in_range( channel, 150.0f, 1000.0f ) )
	
	//重复捕获错误
	//重置捕获
	if( TIM1->SR & (1<<9) )
	{
		//reset all
		old_RC_cnt = TIM1->CCR1;
		set_RC_channel_invalid;
		//clear interrupt flags
		TIM1->SR = 0;
		return;
	}
	
	//清空捕获标志
	TIM1->SR = 0;
	//计算捕获值
	float new_RC_channel = ( (float)count_short( old_RC_cnt , TIM1->CCR1 ) - 1000.0f ) * 0.1f;
	old_RC_cnt = TIM1->CCR1;
	
	//deal with channel value error
	if( new_RC_channel < -20.0f )
	{
		set_RC_channel_invalid;
		return;
	}
	
	if( RC_channel_valid == false )
	{
		if( is_Synchro_Blank( new_RC_channel ) )
			reset_RC_channel;
	}
	else
	{
		if( new_RC_channel < 120.0f )
		{
			if( (current_RC_channel < 8) && (new_RC_channel > -10.0f ) )
			{
				RC_channels_value[ current_RC_channel ] = new_RC_channel;
				constrain( RC_channels_value[ current_RC_channel ], 0.0f, 100.0f );
				++current_RC_channel;
			}
			else
				set_RC_channel_invalid;
		}
		else
		{
			if( is_Synchro_Blank( new_RC_channel ) && ( current_RC_channel >= 6 ) && ( channels_count == current_RC_channel )  )
			{	//接收成功
				channels_count = current_RC_channel;
				reset_RC_channel;			
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				xTimerPendFunctionCallFromISR( TIM1_IRQHandlerTCB, 0, 0, &xHigherPriorityTaskWoken );
				NVIC_DisableIRQ(TIM1_CC_IRQn);
				portYIELD_FROM_ISR( xHigherPriorityTaskWoken );					
			}
			else
			{
				channels_count = current_RC_channel;
				set_RC_channel_invalid;
			}
		}
	}
}

void init_drv_RCPPM()
{
	//GPIO配置 PA8(PPM) 开漏上拉
  RCC->AHB4ENR |= (1<<0);
	os_delay(0.01);
	set_register( GPIOA->MODER, 0b10, 8*2, 2 );
	set_register( GPIOA->PUPDR, 0b01, 8*2, 2 );
  GPIOA->OTYPER |= (1<<8);
	set_register( GPIOA->AFR[1], 0b0001, 8*4-32, 4 );
	 
	//TIM1配置输入捕获
  RCC->APB2ENR|=(1<<0);
	os_delay(0.01);
	TIM1->PSC = (APB2TIMERCLK/1e+6)-1;//1MHZ
	TIM1->ARR = 0xffff;
  TIM1->CCMR1 = (0b0010<<4) | (0b01<<0);
	TIM1->DIER = (1<<1);//使能 CC1 中断	
	TIM1->EGR = (1<<0);
	TIM1->CCER = (1<<0);   //使能捕获
	NVIC_SetPriority(TIM1_CC_IRQn,5);
	NVIC_EnableIRQ(TIM1_CC_IRQn);
	TIM1->CR1 = (1<<7)|(1<<0);//自动重载预装载使能/计数器使能
	
	//接收机注册
	ReceiverRegister("PPM");
}