#include "Basic.hpp"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

#include "FreeRTOS.h"
#include "timers.h"

#define PullUp_Trig ( GPIOB->BSRR = (1<<14) )
#define PullDn_Trig ( GPIOB->BSRR = (1<<30) )

static void UltrasonicTCB( void *pvParameter1, uint32_t ulParameter2 )
{
	//计算超声波测距距离
	vector3<double> position;
	position.z = ulParameter2*2e-6 * 17000.0;
	
	//更新传感器数据	
	if( position.z>1 && position.z<1000 )
	{
		//获取倾角
		Quaternion quat;
		get_Airframe_quat( &quat );
		double lean_cosin = quat.get_lean_angle_cosin();
		//更新
		position.z *= lean_cosin;
		PositionSensorUpdatePosition( default_ultrasonic_sensor_index, position, true );
	}
	else
		PositionSensorSetInavailable( default_ultrasonic_sensor_index );
}

extern "C" void TIM8_BRK_TIM12_IRQHandler()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint32_t SR = TIM12->SR;
	if( SR & (1<<1) )
	{	//捕获到高电平时间
		uint32_t CCR1 = TIM12->CCR1;
		xTimerPendFunctionCallFromISR( UltrasonicTCB, 0, CCR1, &xHigherPriorityTaskWoken );		
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static TimerHandle_t UltrasonicTimer;
static void UltrasonicTimerCallback( TimerHandle_t xTimer )
{
	static uint8_t Ultrasonic_state = 0;
	switch( Ultrasonic_state )
	{
		case 0:
			//发送高电平
			PullUp_Trig;
			Ultrasonic_state = 1;
			xTimerChangePeriod( UltrasonicTimer, 1, 0 );
			break;
		case 1:
			//拉低等待
			PullDn_Trig;
			Ultrasonic_state = 0;
			xTimerChangePeriod( UltrasonicTimer, 0.1*configTICK_RATE_HZ, 0 );
			break;
	}
}

void init_drv_ultrasonic()
{
	/*GPIO配置 PB14(Trig) PB15(Echo)*/
		//打开GPIOB电源
		RCC->AHB4ENR |= (1<<1);
		os_delay(0.01);

		//PB14推挽输出
		set_register( GPIOB->MODER, 1, 14*2, 2 );
		set_register( GPIOB->OSPEEDR, 0, 14*2, 2 );
		set_register( GPIOB->PUPDR, 0, 14*2, 2 );
		
		//PB15复用功能开漏上拉
		set_register( GPIOB->MODER, 2, 15*2, 2 );
		set_register( GPIOB->AFR[1], 2, 15*4-32, 4 );
		GPIOB->OTYPER |= (1<<15);
		set_register( GPIOB->PUPDR, 1, 15*2, 2 );
	/*GPIO配置 PB14(Trig) PB15(Echo)*/
	
	PullDn_Trig;
	
	/*TIM12配置*/
		//打开TIM12电源
		RCC->APB1LENR |= (1<<6);
		os_delay(0.01);
		
		TIM12->PSC = (APB1TIMERCLK/500e+3)-1;	//500khz
		TIM12->ARR = 50000;	//0.1秒周期
		
		TIM12->CCMR1 = (1<<12) | (0b01<<8)  |  (1<<4) | (0b10<<0);
		TIM12->SMCR = (0b110<<4) | (1<<16);
		TIM12->CCER = (0<<3) | (1<<1) | (1<<0);	
		TIM12->EGR = (1<<0);
		TIM12->DIER = (1<<1);
		NVIC_SetPriority(TIM8_BRK_TIM12_IRQn,4);
		NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);		
		TIM12->CR1 = (1<<3) | (1<<0);
	/*TIM12配置*/
	
	//注册传感器
	PositionSensorRegister( default_ultrasonic_sensor_index , \
													Position_Sensor_Type_RangePositioning , \
													Position_Sensor_DataType_s_z , \
													Position_Sensor_frame_ENU , \
													0.05 , //延时
													0 ,	//xy信任度
													0 //z信任度
													);
	
	//定时采样
	UltrasonicTimer = xTimerCreate( "UltraT", 1, pdFALSE, 0, UltrasonicTimerCallback );
	xTimerStart( UltrasonicTimer, 0 );
}