#pragma once

void init_drv_LED(void);

/*
	LED调亮度函数
	R、G、B：亮度百分比（0-100）
*/
void set_LedBrightness( float R , float G , float B );

/*
	蜂鸣器频率调节函数
	freq:蜂鸣器频率
*/
void set_BuzzerFreq( unsigned short freq );

/*
	蜂鸣器鸣响函数
	on:是否鸣响
*/
void set_BuzzerOnOff( bool on );