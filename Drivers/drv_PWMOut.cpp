#include "drv_PwmOut.hpp"
#include "Basic.hpp"
#include "stm32h7xx.h"
#include "TimeBase.hpp"

void PWM_Out(double out[8])
{
	TIM4->CCR3 = out[0]*100 + 10000;
	TIM4->CCR4 = out[1]*100 + 10000;
	TIM8->CCR1 = out[2]*100 + 10000;
	TIM8->CCR2 = out[3]*100 + 10000;
	TIM3->CCR1 = out[4]*100 + 10000;
	TIM3->CCR2 = out[5]*100 + 10000;
	TIM15->CCR1 = out[6]*100 + 10000;
	TIM15->CCR2 = out[7]*100 + 10000;
}
void PWM_PullDownAll()
{
  TIM15->CCR2=10000;
	TIM15->CCR1=10000;
  TIM3->CCR2 =10000;
  TIM3->CCR1 =10000;
  TIM8->CCR2 =10000;
	TIM8->CCR1 =10000;
	TIM4->CCR4 =10000;
	TIM4->CCR3 =10000;
}
void PWM_PullUpAll()
{
	TIM15->CCR2=20000;
	TIM15->CCR1=20000;
  TIM3->CCR2 =20000;
  TIM3->CCR1 =20000;
  TIM8->CCR2 =20000;
	TIM8->CCR1 =20000;
	TIM4->CCR4 =20000;
	TIM4->CCR3 =20000;
}


/*
   PWM1(TIM15_CH2) -- PE6
	 PWM2(TIM15_CH1) -- PE5
	 PWM3( TIM3_CH2) -- PB5
	 PWM4( TIM3_CH1) -- PB4
   PWM5( TIM8_CH2) -- PC7
	 PWM6( TIM8_CH1) -- PC6
	 PWM7( TIM4_CH4) -- PD15
	 PWM8( TIM4_CH3) -- PD14
*/
void init_drv_PWMOut()
{
	/*配置GPIO*/  
	
	//使能GPIO时钟
	RCC->AHB4ENR |= (1<<4)|(1<<1)|(1<<2)|(1<<3);
	os_delay(0.01);
  //复用模式
	set_register( GPIOB->MODER, 0b10, 4*2, 2 );	//PB4
	set_register( GPIOB->MODER, 0b10, 5*2, 2 );	//PB5
	set_register( GPIOC->MODER, 0b10, 6*2, 2 );	//PC6
	set_register( GPIOC->MODER, 0b10, 7*2, 2 );	//PC7
	set_register( GPIOD->MODER, 0b10, 14*2, 2 );	//PD14
	set_register( GPIOD->MODER, 0b10, 15*2, 2 );	//PD15
	set_register( GPIOE->MODER, 0b10, 5*2, 2 );	//PE5
	set_register( GPIOE->MODER, 0b10, 6*2, 2 );	//PE6
	
	//开漏输出
	set_register( GPIOB->OTYPER, 0, 4, 1 );	//PB4
	set_register( GPIOB->OTYPER, 0, 5, 1 );	//PB5
	set_register( GPIOC->OTYPER, 0, 6, 1 );	//PC6
	set_register( GPIOC->OTYPER, 0, 7, 1 );	//PC7
	set_register( GPIOD->OTYPER, 0, 14, 1 );	//PD14
	set_register( GPIOD->OTYPER, 0, 15, 1 );	//PD15
	set_register( GPIOE->OTYPER, 0, 5, 1 );	//PE5
	set_register( GPIOE->OTYPER, 0, 6, 1 );	//PE6
	
	//上拉
	set_register( GPIOB->PUPDR, 0b01, 4*2, 2 );	//PB4
	set_register( GPIOB->PUPDR, 0b01, 5*2, 2 );	//PB5
	set_register( GPIOC->PUPDR, 0b01, 6*2, 2 );	//PC6
	set_register( GPIOC->PUPDR, 0b01, 7*2, 2 );	//PC7
	set_register( GPIOD->PUPDR, 0b01, 14*2, 2 );	//PD14
	set_register( GPIOD->PUPDR, 0b01, 15*2, 2 );	//PD15
	set_register( GPIOE->PUPDR, 0b01, 5*2, 2 );	//PE5
	set_register( GPIOE->PUPDR, 0b01, 6*2, 2 );	//PE6
	
	//复用功能配置
	set_register( GPIOB->AFR[0], 2, 4*4, 4 );	//PB4
	set_register( GPIOB->AFR[0], 2, 5*4, 4 );	//PB5
	set_register( GPIOC->AFR[0], 3, 6*4, 4 );	//PC6
	set_register( GPIOC->AFR[0], 3, 7*4, 4 );	//PC7
	set_register( GPIOD->AFR[1], 2, 14*4-32, 4 );	//PD14
	set_register( GPIOD->AFR[1], 2, 15*4-32, 4 );	//PD15
	set_register( GPIOE->AFR[0], 4, 5*4, 4 );	//PE5
	set_register( GPIOE->AFR[0], 4, 6*4, 4 );	//PE6
	
	
	/*配置TIM*/  	
  //使能TIM时钟
	RCC->APB1LENR|=(1<<2)|(1<<1);
  RCC->APB2ENR|=(1<<16)|(1<<1);
	os_delay(0.01);
	
	PWM_PullDownAll();
	
	TIM3->PSC  =(APB1TIMERCLK / 10e6) - 1;
	TIM3->ARR  = 10e6 / 400;
  TIM3->CCMR1 = (0b110<<12)|(1<<11)|(0b110<<4)|(1<<3);
  TIM3->CCER =(1<<4)|(1<<0);
  TIM3->EGR = (1<<0);
  TIM3->CR1 = (1<<7)|(1<<0);

	TIM4->PSC = (APB1TIMERCLK / 10e6) - 1;
	TIM4->ARR = 10e6 / 400;
	TIM4->CCMR2 = (0b110<<12)|(1<<11)|(0b110<<4)|(1<<3);
	TIM4->CCER =(1<<12) | (1<<8);
	TIM4->EGR |=(1<<0);
	TIM4->CR1 = (1<<7)|(1<<0);
	

  TIM8->PSC = (APB2TIMERCLK / 10e6)-1;
  TIM8->ARR = 10e6 / 400;
  TIM8->CCMR1 = (0b110<<4)|(0b110<<12)|(1<<3)|(1<<11);                                       
	TIM8->CCER = (1<<4)|(1<<0);
	TIM8->BDTR = (1<<15);
  TIM8->EGR = (1<<0);                    
  TIM8->CR1 = (1<<7)|(1<<0);          

  TIM15->PSC=(APB2TIMERCLK / 10e6)-1;
  TIM15->ARR = 10e6 / 400;
  TIM15->CCMR1 = (0b110<<4)|(0b110<<12)|(1<<3)|(1<<11);                                       
	TIM15->CCER = (1<<4)|(1<<0);
	TIM15->BDTR = (1<<15);
  TIM15->EGR = (1<<0);                    
  TIM15->CR1 = (1<<7)|(1<<0);   

   
}