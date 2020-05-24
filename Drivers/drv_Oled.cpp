#include "Basic.hpp"
#include "drv_Oled.hpp"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"

//访问互斥锁
static SemaphoreHandle_t OledMutex = xSemaphoreCreateRecursiveMutex();
//发送完成标志
static EventGroupHandle_t events = xEventGroupCreate();
#define TxCompleteEvent (1<<0)
//DMA缓冲区
#ifdef DCACHE_SIZE
	__attribute__ ((aligned (4))) Static_AXIDMABuf uint8_t scratch_buff[32];
#else
	__attribute__ ((aligned (4))) static uint8_t scratch_buff[32]; 
#endif
//超时时间
#define TIMEOUT 2.0
//设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏
#define USE_HORIZONTAL 0

/*LCD操作*/
	#define OLED_DC_Clr()   GPIOE->BSRR= (1<<27)//DC
	#define OLED_DC_Set()   GPIOE->BSRR= (1<<11)

	#define OLED_RST_Clr()  GPIOE->BSRR = (1<<29)//RES
	#define OLED_RST_Set()  GPIOE->BSRR = (1<<13)

	#define OLED_BLK_Clr()  GPIOB->BSRR = (1<<16)//BLK
	#define OLED_BLK_Set()  GPIOB->BSRR = (1<<0)
/*LCD操作*/

/*SPI通信函数*/
	extern "C" void SPI4_IRQHandler()
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		if( SPI4->SR & (1<<3) )
		{
			SPI4->IFCR = (0b111111111<<3);
			xHigherPriorityTaskWoken = xEventGroupSetBitsFromISR( events, TxCompleteEvent, &xHigherPriorityTaskWoken );
		}
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}

	static void LCD_Writ_Bus( void* buffer, uint32_t size, bool DisableMInc )
	{
		uint32_t total_size = size;
		uint32_t sd_length;	
		while( size > 0 )
		{
			//计算单次发送数量
			if( size > 65472 )
				sd_length = 65472;
			else
				sd_length = size;
			
			//关闭DMA
			DMA1_Stream2->CR &= ~(1<<0);
			//地址递增模式
			if( DisableMInc )
			{
				DMA1_Stream2->CR &= ~(1<<10);
				DMA1_Stream2->CR |= (0b01<<13);
			}
			else
			{
				DMA1_Stream2->CR |= (1<<10);
				DMA1_Stream2->CR &= ~(0b11<<13);
			}
			//关闭SPI外设
			SPI4->CR1 &= ~(1<<0);
			//禁止 Tx DMA
			SPI4->CFG1 &=~(1<<15);
			
			//清空DMA状态
			DMA1->LIFCR = (1<<21)|(1<<20) | (1<<19) | (1<<18) | (1<<17) | (1<<16);
			//设置DMA存储器地址
			if(DisableMInc)
				DMA1_Stream2->M0AR = (uint32_t)buffer;
			else
				DMA1_Stream2->M0AR = (uint32_t)buffer + (total_size - size);
			//清空SPI状态
			SPI4->IFCR = (0b111111111<<3);
			//设置DMA传输数量
			DMA1_Stream2->NDTR = SPI4->CR2 = sd_length;
			//使能DMA
			DMA1_Stream2->CR |= (1<<0);
			//使能 Tx DMA
			SPI4->CFG1 |=(1<<15);
			//打开SPI外设开始传输
			SPI4->CR1|=(1<<0);
			SPI4->CR1|=(1<<9);
			SPI4->IER|=(1<<3);
			//等待DMA传输完成
			xEventGroupWaitBits( events, TxCompleteEvent, pdTRUE, pdFALSE, TIMEOUT*configTICK_RATE_HZ );
			size -= sd_length;
		}
	}
/*SPI通信函数*/
	
/*OLED操作函数*/
	bool Lock_Oled( double Sync_waitTime )
	{
		uint32_t Sync_waitTicks;
		if( Sync_waitTime > 0 )
			Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
		else
			Sync_waitTicks = portMAX_DELAY;
		if( xSemaphoreTakeRecursive(OledMutex,Sync_waitTicks) == pdTRUE )
			return true;
		return false;
	}
	void UnLock_Oled()
	{
		xSemaphoreGiveRecursive(OledMutex);
	}
	
	static bool LCD_Writ_Bus( void* buffer, uint32_t size, double Sync_waitTime = -1 )
	{	
		if( Lock_Oled(Sync_waitTime) )
		{
	#ifdef DCACHE_SIZE	
			if( isDMABuf( (uint32_t)buffer ) )
			{				
	#endif	//非CACHE区域					
				LCD_Writ_Bus( buffer, size, false );
	#ifdef DCACHE_SIZE	
			}
			else if(((uint32_t)buffer & 0x1f) == 0)//32字节对齐
			{
				//先发送可整除部分
				uint32_t part1_size = size & 0xffffffe0;
				if( part1_size > 0 )
				{
					SCB_CleanDCache_by_Addr( (uint32_t*)buffer, part1_size );
					LCD_Writ_Bus( buffer, part1_size, false );
				}
				//后面部分先复制到scratch再发送
				uint32_t part2_size = size - part1_size;
				if( part2_size > 0 )
				{
					memcpy( scratch_buff, &((uint8_t*)buffer)[part1_size], part2_size );
					LCD_Writ_Bus( scratch_buff, part2_size, false );
				}
			}
			else//32字节不对齐
			{
				//先将第一部分复制到scratch进行传输
				uint32_t buffer_base = ((uint32_t)buffer & 0xffffffe0) + 32;
				uint32_t part1_size = buffer_base - (uint32_t)buffer;
				if( part1_size > size )
					part1_size = size;
				memcpy( scratch_buff, buffer, part1_size );
				LCD_Writ_Bus( scratch_buff, part1_size, false );
				
				//第二部分清空D-Cache后发送
				uint32_t part2_size = ( size - part1_size ) & 0xffffffe0;
				if( part2_size > 0 )
				{
					SCB_CleanDCache_by_Addr( (uint32_t*)buffer_base, part2_size );
					LCD_Writ_Bus( (uint32_t*)buffer_base, part2_size, false );
				}
				
				//第三部分先复制到scratch再发送
				uint32_t part3_size = size - part1_size - part2_size;
				if( part3_size > 0 )
				{
					memcpy( scratch_buff, &((uint8_t*)buffer)[part1_size+part2_size], part3_size );
					LCD_Writ_Bus( scratch_buff, part3_size, false );
				}
			}	
	#endif
			UnLock_Oled();
			return true;
		}
		return false;
	} 

	static bool LCD_WR_DATA8( uint8_t dat, double Sync_waitTime = -1 )
	{
		OLED_DC_Set();//写数据
		((uint8_t*)scratch_buff)[0] = dat;
		return LCD_Writ_Bus( &scratch_buff, 1, Sync_waitTime );
	}

	static bool LCD_WR_DATA( uint16_t dat, double Sync_waitTime = -1 )
	{
		OLED_DC_Set();//写数据
		//dat = __REV16(dat);
		((uint16_t*)scratch_buff)[0] = dat;
		return LCD_Writ_Bus( scratch_buff ,2, Sync_waitTime );
	}

	static bool LCD_WR_REG( uint8_t dat, double Sync_waitTime = -1 )
	{
		OLED_DC_Clr();//写命令
		((uint8_t*)scratch_buff)[0] = dat;
		return LCD_Writ_Bus( &scratch_buff, 1, Sync_waitTime );
	}

	/******************************************************************************
				函数说明：设置起始和结束地址
				入口数据：x1,x2 设置列的起始和结束地址
									y1,y2 设置行的起始和结束地址
				返回值：  无
	******************************************************************************/
	static void LCD_Address_Set( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2 )
	{
		if(USE_HORIZONTAL==0)
		{
			LCD_WR_REG(0x2a);//列地址设置
			LCD_WR_DATA(x1);
			LCD_WR_DATA(x2);
			LCD_WR_REG(0x2b);//行地址设置
			LCD_WR_DATA(y1);
			LCD_WR_DATA(y2);
			LCD_WR_REG(0x2c);//储存器写
		}
		else if(USE_HORIZONTAL==1)
		{
			LCD_WR_REG(0x2a);//列地址设置
			LCD_WR_DATA(x1);
			LCD_WR_DATA(x2);
			LCD_WR_REG(0x2b);//行地址设置
			LCD_WR_DATA(y1+80);
			LCD_WR_DATA(y2+80);
			LCD_WR_REG(0x2c);//储存器写
		}
		else if(USE_HORIZONTAL==2)
		{
			LCD_WR_REG(0x2a);//列地址设置
			LCD_WR_DATA(x1);
			LCD_WR_DATA(x2);
			LCD_WR_REG(0x2b);//行地址设置
			LCD_WR_DATA(y1);
			LCD_WR_DATA(y2);
			LCD_WR_REG(0x2c);//储存器写
		}
		else
		{
			LCD_WR_REG(0x2a);//列地址设置
			LCD_WR_DATA(x1+80);
			LCD_WR_DATA(x2+80);
			LCD_WR_REG(0x2b);//行地址设置
			LCD_WR_DATA(y1);
			LCD_WR_DATA(y2);
			LCD_WR_REG(0x2c);//储存器写
		}
	}
	
	bool LCD_Clear( uint16_t Color, double Sync_waitTime )
	{
		if( Lock_Oled(Sync_waitTime) )
		{
			LCD_Address_Set( 0, 0, LCD_W-1, LCD_H-1 );
			OLED_DC_Set();//写数据
			((uint16_t*)scratch_buff)[0] = Color;
			LCD_Writ_Bus( scratch_buff, LCD_W*LCD_H*2, true );
			UnLock_Oled();
			return true;
		}
		return false;
	}

	bool LCD_WritePicture( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, void* pic, double Sync_waitTime )
	{
		if( Lock_Oled(Sync_waitTime) )
		{
			LCD_Address_Set( x1, y1, x2, y2 );
			OLED_DC_Set();//写数据
			LCD_Writ_Bus( pic, (x2-x1+1)*(y2-y1+1)*2 );
			UnLock_Oled();
			return true;
		}
		return false;
	}
	
/*OLED操作函数*/

void init_drv_Oled()
{
	/*GPIO配置*/
		/*
			PE11 -- SPI4_NSS  -- OLCD_DC
			PE12 -- SPI4_SCLK -- OLCD_SCL
			PE13 -- SPI4_MISO -- OLED_REST
			PE14 -- SPI4_MOSI -- OLCD_SDA
			PB0  -- OLCD_BLK  -- OLCD_BLK
		*/
		/*使能GPIOE和GPIOB时钟*/
		RCC->AHB4ENR |= (1<<4);
		os_delay(1e-2);
		RCC->AHB4ENR |= (1<<1);
		os_delay(1e-2);
		
		/*PE12,PE14复用模式*/
		set_register( GPIOE->MODER, 0b10, 12*2, 2 );	//PE12
		set_register( GPIOE->MODER, 0b10, 14*2, 2 );	//PE14
	
		/*PB0,PE11,PE13通用输出模式*/
		set_register( GPIOB->MODER, 0b01, 0*2, 2 );	//PB0
		set_register( GPIOE->MODER, 0b01, 11*2, 2 );	//PE11
		set_register( GPIOE->MODER, 0b01, 13*2, 2 );	//PE13
		
		/*高速输出*/
		set_register( GPIOB->OSPEEDR, 0b11, 0*2, 2 );	//PB0
		set_register( GPIOE->OSPEEDR, 0b11, 11*2, 2 );	//PE11
		set_register( GPIOE->OSPEEDR, 0b11, 12*2, 2 );	//PE12
		set_register( GPIOE->OSPEEDR, 0b11, 13*2, 2 );	//PE13
		set_register( GPIOE->OSPEEDR, 0b11, 14*2, 2 );	//PE14
		
		/*复用功能初始化*/
		set_register( GPIOE->AFR[1], 5, 12*4-32, 4 );	//PE12
		set_register( GPIOE->AFR[1], 5, 14*4-32, 4 );	//PE14
	/*GPIO配置*/
	
	/*SPI4配置*/
		/*SPI4EN：SPI4 外设时钟使能*/
		RCC->APB2ENR|=(1<<13);
		os_delay(1e-2);
		
		SPI4->CR1 = (1<<12) | (1<<11);
		SPI4->CFG1 = (0b010<<28) | (1<<15) | (7<<0);
		SPI4->CFG2 = (1<<31) | (1<<30) | (1<<26) | (1<<25) | (1<<24) | (1<<22) | (1<<17);
		SPI4->IFCR = (1<<9);
		
		NVIC_SetPriority(SPI4_IRQn,5);
		NVIC_EnableIRQ(SPI4_IRQn);
	/*SPI4配置*/
	
	/*DMA配置*/
		/*使能DMA1时钟*/
		RCC->AHB1ENR|=(1<<0);
		os_delay(1e-2);
				
		DMAMUX1_Channel2->CCR = (84<<0);  
		DMA1_Stream2->PAR=(uint32_t)(&SPI4->TXDR);//外设地址
		DMA1_Stream2->CR = (0<<16) | (0<<13) | (1<<10) | (0<<9) | (0b01<<6);
		DMA1_Stream2->FCR = (1<<2) | (3<<0);		
		//清空DMA状态
		DMA1->LIFCR = (1<<21)|(1<<20) | (1<<19) | (1<<18) | (1<<17) | (1<<16);
	/*DMA配置*/
	
	/*OLED初始化*/
		//OLED Reset
		OLED_BLK_Clr();
		OLED_RST_Clr();
		os_delay(0.2);
		OLED_RST_Set();
		os_delay(0.2);
		
		//sleep out
		LCD_WR_REG(0x11); 
		os_delay(0.12);

		//memory data access control
		LCD_WR_REG(0x36);
		if(USE_HORIZONTAL==0)
		{
			LCD_WR_DATA8(0x00);
		}
		else if(USE_HORIZONTAL==1)
		{
			LCD_WR_DATA8(0xC0);
		}
		else if(USE_HORIZONTAL==2)
		{
			LCD_WR_DATA8(0x70);
		}
		else 
		{
			LCD_WR_DATA8(0xA0);
		}
		
		//RGB 5-6-5
		LCD_WR_REG(0x3A); 
		LCD_WR_DATA8(0x65);
		
		//porch setting
		LCD_WR_REG(0xB2);
		LCD_WR_DATA8(0x0C);
		LCD_WR_DATA8(0x0C);
		LCD_WR_DATA8(0x00);
		LCD_WR_DATA8(0x33);
		LCD_WR_DATA8(0x33);
		
		//Gate Control
		LCD_WR_REG(0xB7); 
		LCD_WR_DATA8(0x72);

		//VCOM Setting
		LCD_WR_REG(0xBB);
		LCD_WR_DATA8(0x3D);	//Vcom=1.625v

		//LCM Control
		LCD_WR_REG(0xC0);
		LCD_WR_DATA8(0x2C);

		//VDV and VRH Command Enable
		LCD_WR_REG(0xC2);
		LCD_WR_DATA8(0x01);

		//VRH Set
		LCD_WR_REG(0xC3);
		LCD_WR_DATA8(0x19);   

		//VDV Set
		LCD_WR_REG(0xC4);
		LCD_WR_DATA8(0x20);  

		//Frame Rate Control in Normal Mode
		LCD_WR_REG(0xC6); 
		LCD_WR_DATA8(0x0F);  //60mhz

		//Power Control 1
		LCD_WR_REG(0xD0); 
		LCD_WR_DATA8(0xA4);
		LCD_WR_DATA8(0xA1);

		//Positive Voltage Gamma Control
		LCD_WR_REG(0xE0);
		LCD_WR_DATA8(0xD0);
		LCD_WR_DATA8(0x04);
		LCD_WR_DATA8(0x0D);
		LCD_WR_DATA8(0x11);
		LCD_WR_DATA8(0x13);
		LCD_WR_DATA8(0x2B);
		LCD_WR_DATA8(0x3F);
		LCD_WR_DATA8(0x54);
		LCD_WR_DATA8(0x4C);
		LCD_WR_DATA8(0x18);
		LCD_WR_DATA8(0x0D);
		LCD_WR_DATA8(0x0B);
		LCD_WR_DATA8(0x1F);
		LCD_WR_DATA8(0x23);
		
		//Negative Voltage Gamma Control
		LCD_WR_REG(0xE1);
		LCD_WR_DATA8(0xD0);
		LCD_WR_DATA8(0x04);
		LCD_WR_DATA8(0x0C);
		LCD_WR_DATA8(0x11);
		LCD_WR_DATA8(0x13);
		LCD_WR_DATA8(0x2C);
		LCD_WR_DATA8(0x3F);
		LCD_WR_DATA8(0x44);
		LCD_WR_DATA8(0x51);
		LCD_WR_DATA8(0x2F);
		LCD_WR_DATA8(0x1F);
		LCD_WR_DATA8(0x1F);
		LCD_WR_DATA8(0x20);
		LCD_WR_DATA8(0x23);
		
		//Display Inversion On
		LCD_WR_REG(0x21); 
		LCD_WR_REG(0x29); 
		
		//LSB
		LCD_WR_REG(0xB0);
		LCD_WR_DATA8(0x00);
		LCD_WR_DATA8(1<<3);
		
		LCD_Clear(0xffff);
		OLED_BLK_Set();
	/*OLED初始化*/
}