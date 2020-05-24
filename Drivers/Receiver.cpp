#include "Receiver.hpp"

#include "AC_Math.hpp"
#include "Parameters.hpp"
#include <map>
#include "FreeRTOS.h"
#include "semphr.h"

using namespace std;

/*
	Receiver配置：
	uint64：8xint8_t 8通道映射
	4xfloat：8通道最小值(offset)
	4xfloat：8通道缩放(scale)
*/
struct ReceiverSync
{
	Receiver rc;
	SemaphoreHandle_t mutex;
};
static map<SName,ReceiverSync> Receivers;

//
bool ReceiverRegister( SName name )
{
	LockInitializationStatus();
	if( getInitializationCompleted() == true )
	{
		UnLockInitializationStatus();
		return false;
	}
	
	MAV_PARAM_TYPE param_types[] = { 
		MAV_PARAM_TYPE_UINT64,	//8通道映射
		MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, //8通道最小值
		MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_REAL32, //8通道缩放
	};
	PR_RESULT res = ParamGroupRegister( SName("RC_")+name, 0, 9, param_types, 0, 0 );
	if( res != PR_OK )
	{	//加入参数表失败
		UnLockInitializationStatus();
		return false;
	}
	
	//初始化新接收机
	ReceiverSync new_rc;
	new_rc.rc.available = new_rc.rc.connected = false;
	new_rc.rc.last_update_time = TIME::now();
	new_rc.mutex = xSemaphoreCreateMutex();
	
	//加入接收机表
	vPortEnterCritical();
	Receivers.insert( pair<SName,ReceiverSync>(name,new_rc) );
	vPortExitCritical();
	
	UnLockInitializationStatus();
	return true;
}

//更新接收机数据
bool ReceiverUpdate( SName name, bool connected, float raw_data[16], uint8_t channel_count, double TIMEOUT )
{
	//初始化完成后才可以更新接收机
	if( getInitializationCompleted() == false )
		return false;
	if( channel_count < 6 || channel_count > 16 )
		return false;
	
	map<SName, ReceiverSync>::iterator it = Receivers.find(name);
	if( it == Receivers.end() )
		return false;
	
	//获取接收机配置参数
	bool rc_initialized;
	uint64_t rc_config[9];
	ReadParamGroup( SName("RC_")+name, rc_config, &rc_initialized );
	rc_initialized = !rc_initialized;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake(it->second.mutex,TIMEOUT_Ticks) )
	{
		it->second.rc.connected = connected;
		if(connected)
		{	//已连接更新数据
			it->second.rc.update_time = it->second.rc.last_update_time.get_pass_time_st();
			memcpy( it->second.rc.raw_data, raw_data, channel_count*sizeof(float) );
			if(rc_initialized)
			{	//Receiver已校准更新通道数据
				uint8_t* ch_reflections = (uint8_t*)&rc_config[0];
				float* channels_Min_RC = (float*)&rc_config[1];
				float* channels_RC_Scale = (float*)&rc_config[5];
				it->second.rc.available_channels = 8;
				for( uint8_t i = 0; i < 8; ++i )
				{
					int8_t ch_reflection = ch_reflections[i];
					
					if( ch_reflection < channel_count && raw_data[ch_reflection]>-20 && raw_data[ch_reflection]<120.0f )
						it->second.rc.data[i] = constrain( ( raw_data[ch_reflection] - channels_Min_RC[i] ) * channels_RC_Scale[i] , 0.0f , 100.0f );
					else if( i < 6 )
					{	//通道数目不足
						it->second.rc.available_channels = i;
						it->second.rc.available = false;
						xSemaphoreGive(it->second.mutex);
						return false;
					}
					else
					{
						it->second.rc.available_channels = i;
						break;
					}
				}
				it->second.rc.available = true;
			}
			else
				it->second.rc.available = false;
		}
		else
			it->second.rc.available = false;
		xSemaphoreGive(it->second.mutex);
		return true;		
	}
	return false;
}

/*获取接收机*/
	static SemaphoreHandle_t rc_it_Mutex = xSemaphoreCreateMutex();
	static map<SName, ReceiverSync>::iterator rc_it = Receivers.end();
	/*
		获取首个可用接收机
		rc：获取的接收机
		name：获取到的接收机名称
		TIMEOUT：超时时间
	*/
	bool getReceiver( Receiver* rc, SName* name, double TIMEOUT )
	{
		//初始化完成后才可以获取接收机
		if( getInitializationCompleted() == false )
		{
			rc->available = rc->connected = false;
			return false;
		}
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		
		//先判断rc_it是否可用
		map<SName, ReceiverSync>::iterator it;
		if( xSemaphoreTake( rc_it_Mutex, TIMEOUT_Ticks ) )
		{
			it = rc_it;
			xSemaphoreGive(rc_it_Mutex);
		}
		else
		{
			rc->available = rc->connected = false;
			return false;
		}
		
		if( it != Receivers.end() )
		{
			if( xSemaphoreTake(it->second.mutex,TIMEOUT_Ticks) )
			{
				if( it->second.rc.connected == true )
				{
					if( it->second.rc.last_update_time.get_pass_time() > 2.0f )
					{
						it->second.rc.connected = false;				
						xSemaphoreGive(it->second.mutex);
					}
					else
					{
						*rc = it->second.rc;
						if( name != 0 )
							*name = it->first;
						xSemaphoreGive(it->second.mutex);
						return true;
					}
				}
				xSemaphoreGive(it->second.mutex);
			}
			else
			{
				rc->available = rc->connected = false;
				return false;
			}
		}
		
		//rc_it不可用，遍历接收机寻找可用的
		it = Receivers.begin();
		while( it != Receivers.end() )
		{
			if( xSemaphoreTake(it->second.mutex,TIMEOUT_Ticks) )
			{
				if( it->second.rc.connected == false )
				{			
					xSemaphoreGive(it->second.mutex);
					++it;
					continue;
				}
				if( it->second.rc.last_update_time.get_pass_time() > 2.0f )
				{
					it->second.rc.connected = false;				
					xSemaphoreGive(it->second.mutex);
					++it;
					continue;
				}
				
				*rc = it->second.rc;
				if( name != 0 )
					*name = it->first;
				xSemaphoreGive(it->second.mutex);
				if( xSemaphoreTake( rc_it_Mutex, TIMEOUT_Ticks ) )
				{
					rc_it = it;
					xSemaphoreGive(rc_it_Mutex);
				}
				return true;
			}
			++it;
		}
		//无接收机可用
		//更新rc_it指针
		if( xSemaphoreTake( rc_it_Mutex, TIMEOUT_Ticks ) )
		{
			rc_it = Receivers.end();
			xSemaphoreGive(rc_it_Mutex);
		}
		rc->available = rc->connected = false;
		return false;
	}

	/*
		获取指定接收机
		name：接收机名称
		rc：获取的接收机
		TIMEOUT：超时时间
	*/
	bool getReceiver( SName name, Receiver* rc, double TIMEOUT )
	{
		//初始化完成后才可以获取接收机
		if( getInitializationCompleted() == false )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		map<SName, ReceiverSync>::iterator it = Receivers.find(name);
		if( it != Receivers.end() )
		{
			if( xSemaphoreTake(it->second.mutex,TIMEOUT_Ticks) )
			{
				if( it->second.rc.last_update_time.get_pass_time() > 2.0f )
				{
					it->second.rc.connected = false;
					xSemaphoreGive(it->second.mutex);
					return false;
				}
				
				*rc = it->second.rc;
				xSemaphoreGive(it->second.mutex);
				return true;
			}
			++it;
		}
		return false;
	}
/*获取接收机*/