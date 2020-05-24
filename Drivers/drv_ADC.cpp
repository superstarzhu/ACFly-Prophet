#include "drv_ADC.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"
#include "Basic.hpp"
#include "math.h"
#include "Parameters.hpp"

//adc3通道数
#define ADC_CHANNELS 4
//DMA存储数组,非Cache
Static_AXIDMABuf __attribute__((aligned(4))) uint32_t ADC3ConvertedData[ADC_CHANNELS];
//VREFINT校准值
uint16_t VREFINT_CAL = *(volatile uint16_t*)0x1FF1E860;
//温度校准值
uint16_t TS_CAL1 = *(volatile uint16_t*)0x1FF1E820;
uint16_t TS_CAL2 = *(volatile uint16_t*)0x1FF1E840;

/*
  获取电池电流大小,单位：A
*/
float Get_MainBaterry_Current()
{
	float kp[2];
	if( ReadParam("Bat_CurrentMKp", 0, 0, (uint64_t*)kp, 0 ) == PR_OK )
		return kp[0] * ((float)ADC3ConvertedData[2] * 3.3f / 65535.0f );
	else
		return 0;
}

/*
  获取电池电压,单位；V
*/
float Get_MainBaterry_Voltage()
{
	float kp[2];
	if( ReadParam("Bat_VoltMKp", 0, 0, (uint64_t*)kp, 0 ) == PR_OK )
		return kp[0] * (float)ADC3ConvertedData[1] * 3.3f / 65535.0f;
	else
		return 0;
}

/*
  获取VDDA基准电压,单位；V
*/
float Get_VDDA_Voltage()
{	
	return (3.3f * (float)VREFINT_CAL / (float)ADC3ConvertedData[0]);	
}

/*
  获取温度，单位：℃
*/
float Get_Temperature()
{
	return (110.0f * 30.0f) / (TS_CAL2 - TS_CAL1) * ( (float)ADC3ConvertedData[3] - TS_CAL1 ) + 30.0f;
}



extern "C" void TIM4_IRQHandler()
{
	TIM4->SR = 0;
	ADC3->CR |=(1<<2);	
}

void init_drv_ADC(void)
{
/*
	VREFINT_ADC -- ADC3_INP19
	Battery_ADC -- PC1 ADC3_INP11
	Current_ADC -- PC0 ADC3_INP10
	TEMP_ADC -- ADC3_INP18*/
	
//配置引脚	
	RCC->AHB4ENR |= (1<<2) ;
	delay(1e-2);
	set_register( GPIOC->MODER , 0b11 , 0*2 , 2 );	
	set_register( GPIOC->MODER , 0b11 , 1*2 , 2 );	
	
//配置DMA	
	RCC->AHB1ENR |= (1<<0);
	os_delay(1e-2);
	DMA1_Stream5->PAR = (uint32_t)&ADC3->DR;
	DMAMUX1_Channel5->CCR = (115<<0);
	DMA1_Stream5->CR = (0b1010<<11) | (1<<10) | (1<<8);
	DMA1_Stream5->FCR = (1<<2) | (3<<0);
		
	//配置ADC	
	RCC->AHB4ENR |= (1<<24);
	os_delay(1e-2);
	//ADC退出深度掉电模式,ADC稳压器使能
	ADC3->CR = 0; 
	ADC3->CR = (1<<28);    
	os_delay(1e-2);
	while((ADC3->CR & (1<<28))!=(1<<28)){};
  
	//选择ADC时钟
	ADC3_COMMON->CCR = (1<<23) | (1<<22) | (0b0000<<18) | (0b00<<16);
	ADC3->CR |= (0b11<<8);
	os_delay(1e-2);
		
	//等待单端输入线性校准完成
	ADC3->CR &= ~(1<<30); 
	ADC3->CR |= (1<<16);
  ADC3->CR |= (1<<31); 
	os_delay(1e-2);
  while((ADC3->CR &(1<<31))==(1<<31)){};	
		
	//配置ADC通道	
	ADC3->CFGR = (1<<31) | (1<<14) | (1<<13) | (1<<12) | (0b11<<0);
	ADC3->CFGR2 = (1023<<16) | (10<<5) | (1<<0);
	ADC3->SMPR2 = (0b111<<0) | (0b111<<3) | (0b111<<24) | (0b111<<27);	
	ADC3->PCSEL = (1<<10) | (1<<11) | (1<<18) | (1<<19);
	ADC3->SQR1 = (19<<6) | (11<<12) | (10<<18) | (18<<24) | ((ADC_CHANNELS-1)<<0);
		
	//打开ADC外设
	ADC3->ISR = ADC3->ISR;
	ADC3->CR |= (1<<0);
	os_delay(1e-2);
  while( (ADC3->ISR & (1<<0))!= (1<<0) );
	ADC3->ISR = ADC3->ISR;
//	//使能中断（EOS）
//	ADC3->IER = (1<<3);
//	NVIC_SetPriority(ADC3_IRQn,5);
//	NVIC_EnableIRQ(ADC3_IRQn);
	//开始转换
	DMA1_Stream5->M0AR = (uint32_t)ADC3ConvertedData;
	DMA1_Stream5->NDTR = ADC_CHANNELS;
	DMA1_Stream5->CR |= (1<<0);	
	ADC3->CR |= (1<<2);	
}