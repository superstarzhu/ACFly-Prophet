#include "Basic.hpp"
#include "FreeRTOS.h"
#include "timers.h"
#include "ReceiverBackend.hpp"

void init_drv_RCSbus()
{
	/*GPIO初始化*/
		//PD6(USART2 RX) SBUS引脚
	
		//打开GPIOD时钟
		RCC->AHB4ENR|=(1<<3);
		os_delay(1e-2);
	
		//复用功能开漏上拉
		GPIOD->OTYPER |= (1<<6);
		set_register( GPIOD->PUPDR, 0b01, 6*2 , 2 );
		set_register( GPIOD->AFR[0], 0b0111, 6*4, 4 );
		set_register( GPIOD->MODER, 0b10, 6*2, 2);		
	/*GPIO初始化*/
	
	/*Usart2初始化*/
		//打开Usart2时钟
		RCC->APB1LENR|=(1<<17);
		os_delay(1e-2);
	
		USART2->CR1 = (1<<29) | (1<<26) | (1<<12) | (1<<10) | (1<<2);
		USART2->CR2 = (1<<23) | (1<<16) | (0b10<<12);    //电平反向 2个停止位
		USART2->CR3 = (1<<28) | (0b011<<25);
		USART2->RTOR = 10;
		USART2->BRR = USART234578CLK / 100e+3;
		USART2->CR1 |= (1<<0);    //USART使能
		NVIC_SetPriority(USART2_IRQn,5);
		NVIC_EnableIRQ(USART2_IRQn);
	/*Usart2初始化*/
	
	//接收机注册
	ReceiverRegister("Sbus");
}
static float rc_buf[16];
static bool failsafe;
extern "C" void USART2_IRQHandlerTCB( void *pvParameter1, uint32_t ulParameter2 )
{
	ReceiverUpdate( "Sbus", !failsafe, rc_buf, 16, 0.01 );
	NVIC_EnableIRQ(USART2_IRQn);
}
extern "C" void USART2_IRQHandler()
{
	bool err = false;
	if( USART2->ISR & (1<<3) )
	{
		err = true;
		USART2->ICR = 1<<3;
	}
	if( USART2->ISR & (1<<0) )
	{
		err = true;
		USART2->ICR = 1<<0;
	}
	USART2->ICR = 1<<11;	
	
	static uint8_t SBUS_RC_State = 0;	
	static unsigned char current_rc_channel = 0;
	static unsigned char rc_bit_count = 0;
	static unsigned short current_rc = 0;	
	
	#define SBUS_RC_Reset (SBUS_RC_State = current_rc_channel = rc_bit_count = current_rc = 0)
	while( ( USART2->ISR & (1<<5) ) != 0 )
	{
		uint8_t rdata = USART2->RDR & 0XFF;
		
		if(err)
			SBUS_RC_Reset;
		
		if(SBUS_RC_State == 0)
		{
			if(rdata == 0x0f)
			{
				++SBUS_RC_State;
			}
		}
		else if(SBUS_RC_State<=22)
		{
			rc_bit_count += 8;
			if(rc_bit_count >=11)
			{
				rc_bit_count -= 11;
				unsigned char l_byte_count = 8 - rc_bit_count;
				rc_buf[ current_rc_channel++ ] = 0.04885197850512945774303859306302f *
							(float)( current_rc | ( ((unsigned char)( rdata << rc_bit_count )) << 3 ) );
				current_rc = (unsigned char)( rdata >> l_byte_count );
			}
			else
			{
				current_rc |= rdata << (rc_bit_count - 8);	
			}
			++SBUS_RC_State;
		}else if(SBUS_RC_State == 23)
		{
			//判断是否失控
			#define SBUS_FAILSAFE_BIT	3
			#define SBUS_FRAMELOST_BIT	2
			failsafe = rdata & (1<<SBUS_FAILSAFE_BIT);
			++SBUS_RC_State;
		}
		else
		{
			if( rdata == 0 )
			{			
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				xTimerPendFunctionCallFromISR( USART2_IRQHandlerTCB, 0, 0, &xHigherPriorityTaskWoken );			
				SBUS_RC_Reset;
				NVIC_DisableIRQ(USART2_IRQn);
				portYIELD_FROM_ISR( xHigherPriorityTaskWoken );	
			}
		}
	}
}