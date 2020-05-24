#include "Basic.hpp"

#include <stdlib.h>
#include <stdio.h>
#include "drv_Oled.hpp"
#include "GUI_Images.hpp"
#include "Font.hpp"

#include "MeasurementSystem.hpp"
#include "Sensors.hpp"
#include "Receiver.hpp"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"

//超时时间
#define TIMEOUT 2.0*configTICK_RATE_HZ
//发送完成标志
#define DMA2D_TCIF (1<<1)
static EventGroupHandle_t TC_events = xEventGroupCreate();
//显存
static __attribute__ ((aligned(4))) __attribute__((section(".SRAM1"))) uint16_t ImgBuffer[240*240];
	
/*RGB565颜色*/
	#define  BLACK	0x0000		//黑色
	#define  NAVY		0x000F		//  深蓝色
	#define  DGREEN  0x03E0    // 深绿色
	#define  DCYAN   0x03EF    // 深青色
	#define  MAROON  0x7800    // 深红色
	#define  PURPLE  0x780F    // 紫色
	#define  OLIVE   0x7BE0    // 橄榄绿
	#define  LGRAY   0xC618    // 灰白色
	#define  DGRAY   0x7BEF    // 深灰色
	#define  BLUE   0x001F    // 蓝色
	#define  GREEN   0x07E0    // 绿色
	#define  CYAN   0x07FF    // 青色
	#define  RED    0xF800    // 红色
	#define  MAGENTA  0xF81F    // 品红
	#define  YELLOW  0xFFE0    // 黄色
	#define  WHITE   0xFFFF    // 白色
/*RGB565颜色*/

/*DMA2D图像操作*/
	
	extern "C" void DMA2D_IRQHandler()
	{
		uint32_t ISR = DMA2D->ISR;
		DMA2D->IFCR = ISR & ((1<<5) | (1<<4) | (1<<3) | (1<<2) | (1<<1) | (1<<0));
		BaseType_t HigherPriorityTaskWoken = pdFALSE;
		//置位完成标志
		xEventGroupSetBitsFromISR( TC_events, ISR, &HigherPriorityTaskWoken );	
	}

	enum ImgFormat
	{
		ImgFormat_ARGB8888 = 0b0000 ,
		ImgFormat_RGB888 = 0b0001 ,
		ImgFormat_RGB565 = 0b0010 ,
		ImgFormat_ARGB1555 = 0b0011 ,
		ImgFormat_ARGB4444 = 0b0100 ,
	};

	/*图像填充
		dest：目标图像
		color：填充的颜色
		Format：图像格式	LO：行偏移
	*/
	static void ImageFill( 
			void* dest, ImgFormat DstFormat, uint16_t DstLO,
			uint32_t color, 
			uint16_t height, uint16_t width)
	{
		DMA2D->OCOLR = color;
		DMA2D->OMAR = (uint32_t)dest;
		DMA2D->OPFCCR = (DstFormat<<0);
		DMA2D->NLR = (width<<16) | (height<<0);
		DMA2D->OOR = DstLO;
		DMA2D->CR = (0b11<<16) | (1<<9) | (1<<0);
		
		//等待传输完成
		uint32_t revents = xEventGroupWaitBits( TC_events, DMA2D_TCIF, pdTRUE, pdFALSE, TIMEOUT );
	}
	
	/*图像格式转换
		dest：目标图像	DstFormat：目标图像格式
		FgImg：前景图像
		height：图像行数	width：图像列数
		Format：图像格式	LO：行偏移
		alpha：图像透明度，负数表示不改变
		RBS：红蓝交换
	*/
	static void ImageFormatConversion( 
			void* dest, ImgFormat DstFormat, uint16_t DstLO, 
			void* FgImg,
			uint16_t height, uint16_t width, 
			ImgFormat FgFormat, uint16_t FgLO, 
			int16_t Fg_alpha = -1, bool Fg_RBS = false)
	{
		DMA2D->FGMAR = (uint32_t)FgImg;
		uint32_t RBS = 0;
		if(Fg_RBS)
			RBS = 1<<21;
		if( Fg_alpha >= 0 )
			DMA2D->FGPFCCR = (Fg_alpha<<24) | RBS | (0b10<<16) | (FgFormat<<0);
		else
			DMA2D->FGPFCCR = RBS | (FgFormat<<0);
		DMA2D->OMAR = (uint32_t)dest;
		DMA2D->OPFCCR = (DstFormat<<0);
		DMA2D->NLR = (width<<16) | (height<<0);
		DMA2D->FGOR = FgLO;
		DMA2D->OOR = DstLO;
		DMA2D->CR = (0b01<<16) | (1<<9) | (1<<0);
		
		//等待传输完成
		uint32_t revents = xEventGroupWaitBits( TC_events, DMA2D_TCIF, pdTRUE, pdFALSE, TIMEOUT );
	}
	
	/*图像叠加
		dest：目标图像	DstFormat：目标图像格式
		FgImg：前景图像	BgImg：背景图像
		height：图像行数	width：图像列数
		Format：图像格式	LO：行偏移
		alpha：图像透明度，负数表示不改变
		RBS：红蓝交换
	*/
	static void ImageOverlay( 
			void* dest, ImgFormat DstFormat, uint16_t DstLO, 
			void* FgImg, void* BgImg, 
			uint16_t height, uint16_t width, 
			ImgFormat FgFormat, uint16_t FgLO, 
			ImgFormat BgFormat, uint16_t BgLO, 
			int16_t Fg_alpha = -1, bool Fg_RBS = false,
			int16_t Bg_alpha = -1 )
	{
		DMA2D->FGMAR = (uint32_t)FgImg;
		uint32_t RBS = 0;
		if(Fg_RBS)
			RBS = 1<<21;
		if( Fg_alpha >= 0 )
			DMA2D->FGPFCCR = (Fg_alpha<<24) | RBS | (0b10<<16) | (FgFormat<<0);
		else
			DMA2D->FGPFCCR = RBS | (FgFormat<<0);
		DMA2D->BGMAR = (uint32_t)BgImg;
		if( Bg_alpha >= 0 )
			DMA2D->BGPFCCR = (Bg_alpha<<24) | (0b10<<16) | (BgFormat<<0);
		else
			DMA2D->BGPFCCR = (BgFormat<<0);
		DMA2D->OMAR = (uint32_t)dest;
		DMA2D->OPFCCR = (DstFormat<<0);
		DMA2D->NLR = (width<<16) | (height<<0);
		DMA2D->FGOR = FgLO;
		DMA2D->BGOR = BgLO;
		DMA2D->OOR = DstLO;
		DMA2D->CR = (0b10<<16) | (1<<9) | (1<<0);
		
		//等待传输完成
		uint32_t revents = xEventGroupWaitBits( TC_events, DMA2D_TCIF, pdTRUE, pdFALSE, TIMEOUT );
	}
	
/*DMA2D图像操作*/
	
/*文字操作*/
	
	static void DrawFont16x8( const char* str, uint16_t row, uint16_t column, uint16_t color )
	{
		if( row > 240 - 16 )
			return;
		
		uint8_t str_ind = 0;
		while( column < 240 - 8 )
		{
			uint8_t s = str[str_ind];
			if( s>=29 && s<=126 )				
				s -= 29;
			else
				return;
			
			for( uint8_t i = 0; i < 16; ++i )
			{
				//读取对应字体
				uint8_t f = Font_16x8[s][i];
				uint16_t p = (row+i)*240 + column;
				for( uint8_t k = 0; k < 8; ++k )
				{
					if( (f>>k) & 1 )
						((uint16_t*)ImgBuffer)[p+k] = color;
				}
			}
			++str_ind;
			column += 8;
		}
	}
	static void DrawFont24x12( const char* str, uint16_t row, uint16_t column, uint16_t color )
	{
		if( row > 240 - 24 )
			return;
		
		uint8_t str_ind = 0;
		while( column < 240 - 16 )
		{
			uint8_t s = str[str_ind];
			if( s>=32 && s<=126 )				
				s -= 32;
			else
				return;
			
			for( uint8_t i = 0; i < 24; ++i )
			{
				//读取对应字体
				uint16_t f = ((uint16_t*)Font_24x12[s])[i];
				uint16_t p = (row+i)*240 + column;
				for( uint8_t k = 0; k < 12; ++k )
				{
					if( (f>>k) & 1 )
						((uint16_t*)ImgBuffer)[p+k] = color;
				}
			}
			++str_ind;
			column += 12;
		}
	}
	static void DrawFont32x16( const char* str, uint16_t row, uint16_t column, uint16_t color )
	{
		if( row > 240 - 32 )
			return;
		
		uint8_t str_ind = 0;
		while( column < 240 - 16 )
		{
			uint8_t s = str[str_ind];
			if( s>=32 && s<=126 )				
				s -= 32;
			else
				return;
			
			for( uint8_t i = 0; i < 32; ++i )
			{
				//读取对应字体
				uint16_t f = ((uint16_t*)Font_32x16[s])[i];
				uint16_t p = (row+i)*240 + column;
				for( uint8_t k = 0; k < 16; ++k )
				{
					if( (f>>k) & 1 )
						((uint16_t*)ImgBuffer)[p+k] = color;
				}
			}
			++str_ind;
			column += 16;
		}
	}
	
/*文字操作*/

static void GUI_Server(void* pvParameters)
{
	ImageFormatConversion(
			(void*)ImgBuffer, ImgFormat_RGB565, 0,
			(void*)gImage_BG1,
			240, 240,
			ImgFormat_RGB565, 0);
	DrawFont32x16( "ACFly", 175, 35, 0x0 );
	DrawFont24x12( "Prophet", 205, 90, 0x0 );
	LCD_WritePicture( 0, 0, LCD_W-1, LCD_H-1, (void*)ImgBuffer );		
	
	//准确周期延时
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for( uint16_t i = 0; i < 50; ++i )
	{		
		ImageOverlay( 
				(void*)&(((uint16_t*)ImgBuffer)[20*240+45]), ImgFormat_RGB565, 240-150,
				(void*)gImage_ACFLY, (void*)&(((uint16_t*)gImage_BG1)[20*240+45]), 
				150, 150,
				ImgFormat_ARGB1555, 0,
				ImgFormat_RGB565, 240-150,
				i*255/50);
		
		LCD_WritePicture( 0, 0, LCD_W-1, LCD_H-1, (void*)ImgBuffer );
		vTaskDelayUntil( &xLastWakeTime, 0.03*configTICK_RATE_HZ );
	}
	os_delay(2.0);

	ImageFill(
			(void*)ImgBuffer, ImgFormat_RGB565, 0,
			WHITE,
			240, 240);
	ImageFill(
			(void*)ImgBuffer, ImgFormat_RGB565, 240-100,
			MAGENTA,
			240, 100);
	DrawFont32x16( "M00", 0, 5, WHITE );
	DrawFont16x8( "HthA:  0%", 32, 9, WHITE );
	DrawFont16x8( "Rol:  0.0", 48, 17, WHITE );
	DrawFont16x8( "Pit:  0.0", 64, 17, WHITE );
	DrawFont16x8( "Yaw:  0.0", 80, 17, WHITE );
	DrawFont16x8( "Vz:   0.0", 96, 17, WHITE );
	DrawFont16x8( "HthP:  0%", 112, 9, WHITE );
	DrawFont16x8( "Vx:   0.0", 128, 17, WHITE );
	DrawFont16x8( "Vy:   0.0", 144, 17, WHITE );
	DrawFont16x8( "Mag:", 160, 9, WHITE );	DrawFont16x8( "Rc:x", 160, 62, WHITE );
	DrawFont16x8( "\x1f\x1f\x1f", 176, 17, WHITE );
	DrawFont16x8( "Pos:", 192, 9, WHITE );
	DrawFont16x8( "\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f", 208, 17, WHITE );
	DrawFont16x8( "\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f", 224, 17, WHITE );
	LCD_WritePicture( 0, 0, LCD_W-1, LCD_H-1, (void*)ImgBuffer );
	
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, 0.1*configTICK_RATE_HZ );
		
		//清空数据区
		ImageFill(
			(void*)&ImgBuffer[32*240+49], ImgFormat_RGB565, 240-6*8,
			MAGENTA,
			8*16, 6*8);
			
		char str[50];
		vector3<double> vec;
		/*姿态*/
			Quaternion quat;
			get_AirframeY_quat(&quat);
			//roll
			sprintf( str, "%3.1f", rad2degree(quat.getRoll()) );
			DrawFont16x8( str, 48, 49, WHITE );
			//pitch
			sprintf( str, "%3.1f", rad2degree(quat.getPitch()) );
			DrawFont16x8( str, 64, 49, WHITE );
			//yaw
			sprintf( str, "%3.1f", rad2degree(quat.getYaw()) );
			DrawFont16x8( str, 80, 49, WHITE );
		/*姿态*/
		
		/*Z轴*/
			//Vz
			get_VelocityENU_Ctrl(&vec);
			sprintf( str, "%3.1f", vec.z );
			DrawFont16x8( str, 96, 49, WHITE );
			//AltitudeHealth
			PosSensorHealthInf1 posinfz;			
			double health;
			if( get_Health_Z(&posinfz) )
			{
				health = 100 - (posinfz.NoiseMax-posinfz.NoiseMin)*2;
				if( health < 0 )
					health = 0;
			}
			else
				health = 0;
			sprintf( str, "%3.0f%%", health );
			DrawFont16x8( str, 32, 49, WHITE );
		/*Z轴*/
			
		/*XY轴*/
			//Vx
			sprintf( str, "%3.1f", vec.x );
			DrawFont16x8( str, 128, 49, WHITE );
			//Vy
			sprintf( str, "%3.1f", vec.y );
			DrawFont16x8( str, 144, 49, WHITE );
			//PositionHealth
			PosSensorHealthInf2 posinfxy;
			if( get_Health_XY(&posinfxy) )
			{
				health = 100 - safe_sqrt((posinfxy.NoiseMax-posinfxy.NoiseMin).get_square())*2;
				if( health < 0 )
					health = 0;
			}
			else
				health = 0;
			sprintf( str, "%3.0f%%", health );
			DrawFont16x8( str, 112, 49, WHITE );
		/*XY轴*/
			
		/*接收机状态*/
			//清空数据区
			ImageFill(
				(void*)&ImgBuffer[160*240+86], ImgFormat_RGB565, 240-1*8,
				MAGENTA,
				1*16, 1*8);
				
			Receiver rc;
			getReceiver(&rc);
			if( rc.connected )
				if( rc.available )
					DrawFont16x8( "\x1d", 160, 86, WHITE );
				else
					DrawFont16x8( "\x1e", 160, 86, WHITE );
			else
				DrawFont16x8( "\x1f", 160, 86, WHITE );
		/*接收机状态*/
			
		/*罗盘状态*/
			//清空数据区
			ImageFill(
				(void*)&ImgBuffer[176*240+17], ImgFormat_RGB565, 240-3*8,
				MAGENTA,
				1*16, 3*8);
				
			for( uint8_t i = 0; i < IMU_Sensors_Count; ++i )
			{
				IMU_Sensor sensor;
				if( GetMagnetometer( i, &sensor ) )
					DrawFont16x8( "\x1d", 176, 17+i*8, WHITE );
				else
					DrawFont16x8( "\x1f", 176, 17+i*8, WHITE );
			}
		/*罗盘状态*/
			
		/*位置传感器状态*/
			//清空数据区
			ImageFill(
				(void*)&ImgBuffer[208*240+17], ImgFormat_RGB565, 240-8*8,
				MAGENTA,
				2*16, 8*8);
				
			for( uint8_t i = 0; i < 8; ++i )
			{
				Position_Sensor sensor;
				if( GetPositionSensor( i, &sensor ) )
					if( sensor.available )
						DrawFont16x8( "\x1d", 208, 17+i*8, WHITE );
					else
						DrawFont16x8( "\x1e", 208, 17+i*8, WHITE );
				else
					DrawFont16x8( "\x1f", 208, 17+i*8, WHITE );
			}
			for( uint8_t i = 0; i < 8; ++i )
			{
				Position_Sensor sensor;
				if( GetPositionSensor( i+8, &sensor ) )
					if( sensor.available )
						DrawFont16x8( "\x1d", 224, 17+i*8, WHITE );
					else
						DrawFont16x8( "\x1e", 224, 17+i*8, WHITE );
				else
					DrawFont16x8( "\x1f", 224, 17+i*8, WHITE );
			}
		/*位置传感器状态*/
			
		LCD_WritePicture( 0, 0, LCD_W-1, LCD_H-1, (void*)ImgBuffer );
	}
}

void init_GUI()
{
	//打开DMA2D
	RCC->AHB3ENR |= (1<<4);
	//打开SRAM1用作显存
	RCC->AHB2ENR |= (1<<29);
	os_delay(0.01);
	//设置DMA2D AXI带宽
	DMA2D->AMTCR = (5<<8) | (1<<0);
	//打开DMA2D中断
	NVIC_SetPriority(DMA2D_IRQn,5);
	NVIC_EnableIRQ(DMA2D_IRQn);
	
	xTaskCreate( GUI_Server, "GUI", 1024, NULL, SysPriority_UserTask, NULL);
}