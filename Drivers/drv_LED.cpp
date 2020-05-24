#include "stm32h743xx.h"
#include "drv_LED.hpp"
#include "Basic.hpp"
#include "AC_Math.hpp"
/*
	LED调亮度函数
	R、G、B：亮度百分比（0-100）
*/
void set_LedBrightness( float R , float G , float B )
{
	if( B >= 0 && B <= 100 )
		TIM2->CCR2 = pow(B/100,3)*TIM2->ARR;
	if( G >= 0 && G <= 100 )
		TIM2->CCR3 = pow(G/100,3)*TIM2->ARR;
	if( R >= 0 && R <= 100 )
		TIM2->CCR4 = pow(R/100,3)*TIM2->ARR;
}

/*
	蜂鸣器频率调节函数
	freq:蜂鸣器频率
*/
void set_BuzzerFreq( unsigned short freq )
{
	float B = (float)TIM2->CCR2/(float)TIM2->ARR;
	float G = (float)TIM2->CCR3/(float)TIM2->ARR;
	float R = (float)TIM2->CCR4/(float)TIM2->ARR;
	
	if( freq < 200 )
		freq = 200;
	TIM2->ARR = 10e6 / freq;
	if( TIM2->CCR1 != 0 )
		TIM2->CCR1 = TIM2->ARR / 2;
	
	TIM2->CCR2 = B*TIM2->ARR;
	TIM2->CCR3 = G*TIM2->ARR;
	TIM2->CCR4 = R*TIM2->ARR;
}

/*
	蜂鸣器鸣响函数
	on:是否鸣响
*/
void set_BuzzerOnOff( bool on )
{
	if( on )
		TIM2->CCR1 = TIM2->ARR / 2;
	else
		TIM2->CCR1 = 0;
}

void init_drv_LED(void)
{
	/*
		Buzzer(TIM2_CH1) PA0
		LED_B(TIM2_CH2) PA1
		LED_G(TIM2_CH3) PA2
		LED_R(TIM2_CH4) PA3
	*/
	
	//开启GPIOA外设时钟
	RCC->AHB4ENR|=(1<<0);
	os_delay(1e-2);
  
	//配置引脚复用模式
	set_register( GPIOA->MODER , 0b10 , 0 , 2 );	//PA0
	set_register( GPIOA->MODER , 0b10 , 2 , 2 );	//PA1
	set_register( GPIOA->MODER , 0b10 , 4 , 2 );	//PA2
	set_register( GPIOA->MODER , 0b10 , 6 , 2 );	//PA3
	
	//蜂鸣器推挽，LED开漏上拉
	GPIOA->OTYPER |= (0<<0) | (1<<1) | (1<<2) | (1<<3);
	set_register( GPIOA->PUPDR , 1 , 2 , 2 );	//PA1
	set_register( GPIOA->PUPDR , 1 , 4 , 2 );	//PA2
	set_register( GPIOA->PUPDR , 1 , 6 , 2 );	//PA3
	
	//配置引脚TIM2复用
	set_register( GPIOA->AFR[0] , 1 , 0 , 4 );	//PA0
	set_register( GPIOA->AFR[0] , 1 , 4 , 4 );	//PA1
	set_register( GPIOA->AFR[0] , 1 , 8 , 4 );	//PA2
	set_register( GPIOA->AFR[0] , 1 , 12 , 4 );	//PA3
	
	//开启TIM2外设时钟
	RCC->APB1LENR|=(1<<0);
	os_delay(1e-2);

	//计数器频率10mhz
	TIM2->PSC = (APB1TIMERCLK / 10000000) - 1;
	
	//设定计数器重装载值
	TIM2->ARR = 10e6 / 1000;
	
	//配置PWM1模式
	TIM2->CCMR1 = (0b111<<12)|(1<<11)|(0<<8) | (0b110<<4)|(1<<3)|(0<<0);
	TIM2->CCMR2 = (0b111<<12)|(1<<11)|(0<<8) | (0b111<<4)|(1<<3)|(0<<0);
	
	//复位输出
	TIM2->CCR1 = TIM2->CCR2 = TIM2->CCR3 = TIM2->CCR4 = 0;
	
	//开启定时器
	TIM2->CCER = (1<<12) | (1<<8) | (1<<4) | (1<<0);
	TIM2->CR1 = (1<<7)|(1<<0);
}







