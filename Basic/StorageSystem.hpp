#pragma once

#include <stdint.h>
#include "Sensors.hpp"

//存储操作结果
enum SS_RESULT
{
	SS_OK = 0 ,
	SS_ERR ,
	SS_TimeOut ,
};

/*内部存储*/
	/*配置存储*/
		/*
			保存存储文件
			group_name：文件组别名称
			name：文件名称
			content：内容
			length：内容长度
			TIMEOUT：线程同步超时时间
		*/
		SS_RESULT InternalStorage_SaveFile( const char* group_name, const char* name , const void* content , uint32_t length , double TIMEOUT = -1 );
		
		/*
			读取存储文件大小
			group_name：文件组别名称
			name：文件名称
			size：读取的文件大小
			TIMEOUT：线程同步超时时间
		*/
		SS_RESULT InternalStorage_GetFileSize( const char* group_name, const char* name , uint32_t* size , double TIMEOUT = -1 );
		
		/*
			读取存储文件
			group_name：文件组别名称
			name：文件名称
			content：读取的内容
			length：读取的文件长度
			TIMEOUT：线程同步超时时间
		*/
		SS_RESULT InternalStorage_ReadFile( const char* group_name, const char* name , void* content , uint32_t* length , double TIMEOUT = -1 );
	/*配置存储*/
	
	void init_InternalStorage();
/*内部存储*/

/*外部存储*/
	/*Log数据*/		
		/*
			Log格式：
				'A''C' + 主版本(0) + 次版本(1)+数据格式(bit0=0:float bit0=1:double) + 24字节描述
			数据帧：
				'A''C' + 消息类型 + 消息包长度 + (uint32_t)运行时间(100us) + (double)具体数据
		*/
		enum LogMsg
		{
			/*传感器数据*/
			LogMsg_PosSensor = 28 ,
			/*欧拉角姿态：roll pitch yaw rollspeed pitchspeed yawspeed*/
			LogMsg_Attitude = 30 ,
			/*四元数姿态：q0 q1 q2 q3 rollspeed pitchspeed yawspeed*/
			LogMsg_AttitudeQuaternion = 31, 
			/*位置速度：posx posy posz xspeed yspeed zspeed (accx accy accz) （保留：水平传感器(int8_t) + 垂直传感器(int8_t)*/
			LogMsg_LocalPositionNed = 32, 
			
			/*Debug向量*/
			LogMsg_DebugVect = 250, 
		};
	
		bool SDLog_Msg_PosSensor( uint8_t ind, Position_Sensor sensor, double SyncTIMEOUT = -1 );
		bool SDLog_Msg_Attitude( double SyncTIMEOUT = -1 );
		bool SDLog_Msg_AttitudeQuaternion( double SyncTIMEOUT = -1 );
		bool SDLog_Msg_LocalPositionNed( double SyncTIMEOUT = -1 );
		
		bool SDLog_Msg_DebugVect( const char* name, double vect[], uint8_t length, double SyncTIMEOUT = -1 );
	
	void init_SDStorage();
/*外部存储*/
