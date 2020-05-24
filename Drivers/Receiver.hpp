#pragma once

#include "Basic.hpp"
#include <stdbool.h>

//接收机定义
struct Receiver
{
	bool connected;	//是否已连接
	bool available;	//是否可用
	uint8_t available_channels;	//可用的通道数目
	TIME last_update_time;	//上次更新时间
	double update_time;	//更新时间间隔
	
	float raw_data[16];	//原始数据
	float data[8];	//校准后的数据
};
/*
	获取首个可用接收机
	rc：获取的接收机
	name：获取接收机名称（为0不获取）
	TIMEOUT：超时时间
*/
bool getReceiver( Receiver* rc, SName* name = 0, double TIMEOUT = -1 );
	
/*
	获取指定接收机
	name：接收机名称
	rc：获取的接收机
	TIMEOUT：超时时间
*/
bool getReceiver( SName name, Receiver* rc, double TIMEOUT = -1 );