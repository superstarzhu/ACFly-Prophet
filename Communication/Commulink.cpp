#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "drv_LED.hpp"
#include "mavlink.h"
#include "MavlinksendFuncs.hpp"
#include "MavlinkRCProcess.hpp"
#include "MeasurementSystem.hpp"
#include "Sensors.hpp"
#include "Parameters.hpp"
#include <map>

using namespace std;

/*声光提示*/
	static float ledSignalCounter = -1;	static LEDSignal ledSignal;
	static LEDMode ledmode = LEDMode_Processing1;
	static float ledR = 0, ledG = 0, ledB = 0;
	static bool buzzerOn = false;	static uint16_t buzzerFreq;

	void sendLedSignal( LEDSignal signal )
	{
		ledSignalCounter = 0;
		ledSignal = signal;
	}
	void setLedMode( LEDMode mode )
	{
		ledmode = mode;
	}
	void setLedManualCtrl( float R, float G, float B, bool BuzzerOn, uint16_t BuzzerFreq )
	{
		ledR = R;	ledG = G;	ledB = B;
		buzzerOn = BuzzerOn;	buzzerFreq = BuzzerFreq;
		ledmode = LEDMode_Manual;
	}
	static inline void LEDRefresh(float dt)
	{
		if( ledSignalCounter >= 0 )
		{
			switch(ledSignal)
			{
				case LEDSignal_Start1:
				{
					if( ledSignalCounter > 0.3 )
					{
						ledSignalCounter = -1;
						return;
					}
					if( ledSignalCounter < 0.15f )
					{
						set_BuzzerFreq(900);
						set_BuzzerOnOff(true);
						set_LedBrightness( 0, 0, 100 );
					}
					else
					{
						set_BuzzerFreq(1500);
						set_BuzzerOnOff(true);
						set_LedBrightness( 0, 100, 0 );
					}
					break;
				}
				case LEDSignal_Start2:
				{
					if( ledSignalCounter > 0.45 )
					{
						ledSignalCounter = -1;
						return;
					}
					if( ledSignalCounter < 0.15 )
					{
						set_BuzzerFreq(800);
						set_BuzzerOnOff(true);
						set_LedBrightness( 100, 0, 0 );
					}
					else if( ledSignalCounter < 0.3 )
					{
						set_BuzzerFreq(1000);
						set_BuzzerOnOff(true);
						set_LedBrightness( 0, 100, 0 );
					}
					else
					{
						set_BuzzerFreq(1200);
						set_BuzzerOnOff(true);
						set_LedBrightness( 0, 0, 100 );
					}
					break;
				}
				
				case LEDSignal_Continue1:
				{
					if( ledSignalCounter > 0.8 )
					{
						ledSignalCounter = -1;
						return;
					}
					set_BuzzerFreq(1500);				
					if( ledSignalCounter < 0.2f )
					{
						set_LedBrightness( 0, 0, 100 );
						set_BuzzerOnOff(true);
					}
					else if( ledSignalCounter < 0.4f )
					{
						set_LedBrightness( 0, 0, 0 );
						set_BuzzerOnOff(false);
					}
					else if( ledSignalCounter < 0.6f )
						set_LedBrightness( 0, 0, 100 );
					else
						set_LedBrightness( 0, 0, 0 );
					break;
				}
				
				case LEDSignal_Success1:
				{
					if( ledSignalCounter > 0.8 )
					{
						ledSignalCounter = -1;
						return;
					}
					set_BuzzerFreq(1500);
					if( ledSignalCounter < 0.2f )
					{						
						set_LedBrightness( 0, 100, 0 );
						set_BuzzerOnOff(true);
					}
					else if( ledSignalCounter < 0.4f )
					{
						set_LedBrightness( 0, 0, 0 );
						set_BuzzerOnOff(false);
					}
					else if( ledSignalCounter < 0.6f )
					{
						set_LedBrightness( 0, 100, 0 );
						set_BuzzerOnOff(true);
					}
					else
					{
						set_LedBrightness( 0, 0, 0 );
						set_BuzzerOnOff(false);
					}
					break;
				}
				
				
				case LEDSignal_Err1:
				{
					if( ledSignalCounter > 1.0 )
					{
						ledSignalCounter = -1;
						return;
					}
					set_BuzzerFreq(800);
					set_BuzzerOnOff(true);
					if( ledSignalCounter < 0.25f )
						set_LedBrightness( 100, 0, 0 );
					else if( ledSignalCounter < 0.5f )
						set_LedBrightness( 0, 0, 0 );
					else if( ledSignalCounter < 0.75f )
						set_LedBrightness( 100, 0, 0 );
					else
						set_LedBrightness( 0, 0, 0 );
					break;
				}
			}
			ledSignalCounter += dt;
			return;
		}
		
		static float counter = 0;
		switch(ledmode)
		{
			/*正常模式*/
				case LEDMode_Normal1:
				{
					if( counter > 2 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 1 )
						set_LedBrightness( 0 , counter*100 , 0 );
					else
						set_LedBrightness( 0 , 200 - counter*100 , 0 );
					break;
				}
			/*正常模式*/
				
			/*飞行模式*/
				case LEDMode_Flying1:
				{
					if( counter > 1.4 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 1 )
						set_LedBrightness( 0 , counter*30 , 30-counter*30 );
					else if( counter < 1.1 )
						set_LedBrightness( 0 , 0 , 100 );
					else if( counter < 1.2 )
						set_LedBrightness( 0 , 0 , 0 );
					else if( counter < 1.3 )
						set_LedBrightness( 0 , 0 , 100 );
					else
						set_LedBrightness( 0 , 0 , 0 );
					break;
				}
			/*飞行模式*/	
				
			/*处理中*/
				case LEDMode_Processing1:
				{
					if( counter > 0.5 )
						counter = 0;
					set_BuzzerOnOff(false);
					set_LedBrightness( 0 , 0 , counter*200 );
					break;
				}
				case LEDMode_Processing2:
				{
					if( counter > 0.5 )
						counter = 0;
					set_BuzzerOnOff(false);
					if( counter < 0.25 )
						set_LedBrightness( 0 , 100-counter*400 , counter*400 );
					else
						set_LedBrightness( 0 , (counter-0.25)*400 , 100-(counter-0.25)*400 );
					break;
				}
			/*处理中*/
				
			default:
			{	//用户手动控制
				if(buzzerOn)
					set_BuzzerFreq(buzzerFreq);
				set_BuzzerOnOff(buzzerOn);
				set_LedBrightness( ledR, ledG, ledB );
				break;
			}
		}
		counter += dt;
	}
/*声光提示*/

/*通信端口*/
	//端口
	static Port CommuPorts[ MAVLINK_COMM_NUM_BUFFERS ] = {0};
	//发送消息列表
	struct SDMsg
	{
		uint16_t counter;
		uint16_t rate;
	};
	static map<uint16_t,SDMsg> SDMessages[MAVLINK_COMM_NUM_BUFFERS];
	static SemaphoreHandle_t SDMessagesMutex[MAVLINK_COMM_NUM_BUFFERS];
	
	//在指定端口设置消息速率
	bool SetMsgRate( uint8_t port_index, uint16_t Msg, uint16_t RateHz, double TIMEOUT )
	{
		if( port_index >= MAVLINK_COMM_NUM_BUFFERS )
			return false;
		if( Msg >= Mavlink_Send_Funcs_Count )
			return false;
		if( Mavlink_Send_Funcs[ Msg ] == 0 )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( SDMessagesMutex[port_index], TIMEOUT_Ticks ) == pdTRUE )
		{
			uint16_t Rate = 100.0f / RateHz;
			if( Rate==0 && RateHz!=0 )
				Rate = 1;
			map<uint16_t,SDMsg>::iterator it = SDMessages[port_index].find(Msg);
			if( it == SDMessages[port_index].end() )
			{	//无此消息 添加消息
				if( Rate != 0 )
				{
					SDMsg sdmsg;
					sdmsg.rate = Rate;
					sdmsg.counter = 0;
					SDMessages[port_index].insert( pair<uint16_t,SDMsg>(Msg, sdmsg) );
				}
			}
			else
			{	//消息存在 更改速率
				if( Rate != 0 )
					it->second.rate = Rate;
				else
					SDMessages[port_index].erase(it);
			}
			
			xSemaphoreGive(SDMessagesMutex[port_index]);
			return true;
		}
		return false;
	}
	
	//在指定端口发送消息列表
	static bool sendParamListReset = false;
	void sendParamList()
	{
		sendParamListReset = true;
		ResetParametersIterator();
	}
	
	//注册端口用于协议通信
	bool CommuPortRegister( Port port )
	{
		if( port.read == 0 )
			return false;
		if( port.write!=0 && (port.lock==0 || port.unlock==0) )
			return false;
		
		//寻找可用的位置
		int8_t p_index = -1;
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{
			if( CommuPorts[ i ].read == 0 && CommuPorts[ i ].write == 0 )
			{
				p_index = i;
				break;
			}
		}
		//放满了
		if( p_index < 0 )
			return false;
		
		mavlink_init_chan( p_index );
		CommuPorts[ p_index ] = port;
		mavlink_set_proto_version( p_index , 1 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_ATTITUDE, 10 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_GPS_RAW_INT, 5 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5 );
		SetMsgRate( p_index, MAVLINK_MSG_ID_SYS_STATUS, 1 );
		return true;
	}
	//获取端口
	const Port* get_Port( uint8_t port )
	{
		if( port < MAVLINK_COMM_NUM_BUFFERS )
			return &CommuPorts[port];
		else
			return 0;
	}
/*通信端口*/
	
static void Commulink_Server(void* pvParameters)
{
	//初始化开始声音
	sendLedSignal(LEDSignal_Start1);
	//等待初始化完成
	while( getInitializationCompleted() == false )
	{
		//刷新led声光提示
		LEDRefresh(0.01f);
		os_delay(0.01);
	}

	//心跳包计数器
	uint16_t HeartBeat_counter = 0;
	
	//准确周期延时
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//刷新led声光提示
		LEDRefresh(0.01f);

		bool sendHB = false;
		if( ++HeartBeat_counter >= 100 )
		{
			HeartBeat_counter = 0;
			sendHB = true;
		}
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{	//遍历所有端口
			mavlink_message_t msg_sd;
			if( CommuPorts[i].write != 0 )
			{	//
				if( sendHB )
				{	//发送心跳包
					if( mavlink_lock_chan(i,0.01) )
					{
						mavlink_msg_heartbeat_pack_chan( 
							1 ,	//system id
							MAV_COMP_ID_AUTOPILOT1 ,	//component id
							i	,	//chan
							&msg_sd,
							MAV_TYPE_QUADROTOR ,	//type
							MAV_AUTOPILOT_GENERIC ,	//autopilot
							MAV_MODE_PREFLIGHT ,	//base mode
							0xffff ,	//custom mode
							MAV_STATE_STANDBY	//sys status
						);
						mavlink_msg_to_send_buffer(CommuPorts[i].write, 
																			 CommuPorts[i].lock,
																			 CommuPorts[i].unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(i);
					}
				}
				
				/*发送消息列表中的消息*/
					#define MAX_SDMsgs 20
					uint16_t sdmsgs[MAX_SDMsgs];
					uint16_t sdmsgs_count = 0;
					if( xSemaphoreTake( SDMessagesMutex[i], 0.01*configTICK_RATE_HZ ) == pdTRUE )
					{					
						for( map<uint16_t,SDMsg>::iterator it = SDMessages[i].begin(); it != SDMessages[i].end(); ++it )
						{
							if( ++(it->second.counter) >= it->second.rate )
							{
								it->second.counter = 0;
								if( sdmsgs_count < MAX_SDMsgs )
									sdmsgs[sdmsgs_count++] = it->first;
								else
									break;
							}
						}
						xSemaphoreGive(SDMessagesMutex[i]);
					}
					for( uint16_t k = 0; k < sdmsgs_count; ++k )
					{
						if( sdmsgs[k]<Mavlink_Send_Funcs_Count && Mavlink_Send_Funcs[sdmsgs[k]]!=0 )
						{
							if( mavlink_lock_chan( i, 0.01 ) )
							{
								if( Mavlink_Send_Funcs[sdmsgs[k]]( i, &msg_sd ) )
									mavlink_msg_to_send_buffer(CommuPorts[i].write, 
																				 CommuPorts[i].lock,
																				 CommuPorts[i].unlock,
																				 &msg_sd, 0, 0.01);
								mavlink_unlock_chan(i);
							}
						}
					}
				/*发送消息列表中的消息*/
				
				/*发送参数列表*/
					uint32_t param_ind; SName param_name; MAV_PARAM_TYPE param_type; uint64_t param_value;
					if( ReadCurrentParameter( &param_name, &param_ind, &param_type, &param_value, 0 ) == PR_OK )
					{
						//参数名
						char pname[17];
						param_name.get_CharStr(pname);
						//参数值
						float value = *(float*)&param_value;
						//参数个数
						uint32_t params_count;
						GetParametersCount(&params_count);
						
						if( mavlink_lock_chan( i, 0.01 ) )
						{
							mavlink_msg_param_value_pack_chan( 
								1 ,	//system id
								MAV_COMP_ID_AUTOPILOT1 ,	//component id
								i ,	//chan
								&msg_sd,
								pname,	//param id
								value ,	//param value
								param_type ,	//param type
								params_count ,	//param count
								param_ind	//param index
							);
							mavlink_msg_to_send_buffer(CommuPorts[i].write, 
																				 CommuPorts[i].lock,
																				 CommuPorts[i].unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(i);
						}
					}
				}
			/*发送参数列表*/
			
			/*发送航点请求*/
				//任务超时再次请求变量
				extern bool RqMissionInt[MAVLINK_COMM_NUM_BUFFERS];
				extern int32_t RqMissionInd[MAVLINK_COMM_NUM_BUFFERS];
				extern int32_t RqMissionCounter[MAVLINK_COMM_NUM_BUFFERS];
				extern uint8_t RqMissiontarget_sysid[MAVLINK_COMM_NUM_BUFFERS];
				extern uint8_t RqMissiontarget_compid[MAVLINK_COMM_NUM_BUFFERS];
				
				for( uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i )
				{
					if( RqMissionCounter[i] > 0 )
					{
						--RqMissionCounter[i];
						if( (RqMissionCounter[i] & 0xf) == 0 )
						{	//超时发送请求
							if( RqMissionInd[i] == 0 )
							{	//0号航点同时发送int和普通请求
								const Port* port = get_Port(i);
								if( port->write != 0 )
								{
									mavlink_message_t msg_sd;
									mavlink_lock_chan( i, 0.01 );
									mavlink_msg_mission_request_int_pack_chan( 
										1 ,	//system id
										MAV_COMP_ID_AUTOPILOT1 ,	//component id
										i ,	//chan
										&msg_sd,
										RqMissiontarget_sysid[i] ,	//target system
										RqMissiontarget_compid[i] ,	//target component
										0 ,	//seq
										MAV_MISSION_TYPE_MISSION	//mission type
									
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(i);
									
									mavlink_lock_chan( i, 0.01 );
									mavlink_msg_mission_request_pack_chan( 
										1 ,	//system id
										MAV_COMP_ID_AUTOPILOT1 ,	//component id
										i ,	//chan
										&msg_sd,
										RqMissiontarget_sysid[i] ,	//target system
										RqMissiontarget_compid[i] ,	//target component
										0 ,	//seq
										MAV_MISSION_TYPE_MISSION	//mission type
									
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(i);
								}
							}
							else if( RqMissionInt[i] )
							{
								const Port* port = get_Port(i);
								if( port->write != 0 )
								{
									mavlink_message_t msg_sd;
									mavlink_lock_chan( i, 0.01 );
									mavlink_msg_mission_request_int_pack_chan( 
										1 ,	//system id
										MAV_COMP_ID_AUTOPILOT1 ,	//component id
										i ,	//chan
										&msg_sd,
										RqMissiontarget_sysid[i] ,	//target system
										RqMissiontarget_compid[i] ,	//target component
										RqMissionInd[i] ,	//seq
										MAV_MISSION_TYPE_MISSION	//mission type
									
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(i);
								}
							}
							else
							{
								const Port* port = get_Port(i);
								if( port->write != 0 )
								{
									mavlink_message_t msg_sd;									
									mavlink_lock_chan( i, 0.01 );
									mavlink_msg_mission_request_pack_chan( 
										1 ,	//system id
										MAV_COMP_ID_AUTOPILOT1 ,	//component id
										i ,	//chan
										&msg_sd,
										RqMissiontarget_sysid[i] ,	//target system
										RqMissiontarget_compid[i] ,	//target component
										RqMissionInd[i] ,	//seq
										MAV_MISSION_TYPE_MISSION	//mission type
									
									);
									mavlink_msg_to_send_buffer(port->write, 
																						 port->lock,
																						 port->unlock,
																						 &msg_sd, 0, 0.01);
									mavlink_unlock_chan(i);
								}
							}
						}
					}
				}
			/*发送航点请求*/
				
			if( CommuPorts[i].read != 0 )
			{	//接收数据处理
				//每次接收64个字节进行处理
				mavlink_message_t msg;
				uint8_t buf[64];
				uint8_t length;			
				do
				{
					length = CommuPorts[i].read( buf, 64, 0, 0.01 );
					for( uint8_t k = 0; k < length; ++k )
					{
						//消息解包
						if( mavlink_parse_char( i, buf[k], &msg, NULL ) )
						{
							//消息解包完成
							
							//如果消息处理函数存在
							//处理消息
							if( msg.msgid < Mavlink_RC_Process_Count )
							{
								if( Mavlink_RC_Process[ msg.msgid ] != 0 )
									Mavlink_RC_Process[ msg.msgid ]( i , &msg );
							}
						}
					}
				}while( length > 10 );
			}
		}
		if( sendParamListReset == false )
			ParameterIteratorMoveNext();
		else
			sendParamListReset = false;
		vTaskDelayUntil( &xLastWakeTime, 0.01*configTICK_RATE_HZ );
	}
}

void init_Commulink()
{
	for( uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i )
	{
		SDMessagesMutex[i] = xSemaphoreCreateMutex();
	}
	xTaskCreate( Commulink_Server, "Commulink", 2048, NULL, SysPriority_UserTask, NULL);
}