/*
	MSafe安全模式
	用户模式一定时间不进行控制
	或调用enter_MSafe会进入此模式
	
	进入安全模式后（ForceMSafeCtrl）
	用户模式控制将失效
	
	用户模式同时进行水平（XY位置或姿态）
	和垂直高度控制可退出MSafe获得控制权
	
	！！不建议一般用户更改此文件！！
	！！此文件需经过验证再发布！！
*/

#include "Basic.hpp"
#include "Modes.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "Receiver.hpp"
#include "ControlSystem.hpp"
#include "ctrl_Main.hpp"
#include "MeasurementSystem.hpp"
#include "NavCmdProcess.hpp"
#include "Parameters.hpp"

//安全模式任务句柄
TaskHandle_t MSafeTaskHandle;

//安全模式参数
struct MSafeCfg
{
	//自动返航模式
	//0-不自动返航
	//1-自动模式（自动判断电量不足返航）
	uint8_t SfRtMode[8];
	
	//返航速度
	float RtSpeed[2];
	
	//经纬度定位升高返航范围
	float GbRtHRange[2];
	//本地定位（无经纬度）升高返航范围
	float LcRtHRange[2];
	
	//经纬度定位升高高度（对地）
	float GbRtHeight[2];
	//本地定位（无经纬度）升高高度（对地）
	float LcRtHeight[2];
}__PACKED;

#define MSafeRate 20
static void MSafe_Server(void* pvParameters)
{
	//准确周期延时
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	//是否刚进入自动模式
	//16-32有定位全自动模式
	//32-48无定位自动模式
	uint8_t firstAuto = 0;
	NavCmdInf navInf;
	init_NavCmdInf(&navInf);
	uint16_t current_mission_ind;
	//返航高度
	double RtHeight = -1;
	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, (1.0/MSafeRate)*configTICK_RATE_HZ );
		
		//获取是否打开控制器
		//控制器没打开不进行进一步操作
		bool attCtrlEna;
		is_Attitude_Control_Enabled(&attCtrlEna);
		if( attCtrlEna==false )
		{
			//退出安全模式
			ForceMSafeCtrl = false;
			//复位自动控制计时器
			firstAuto = 0;
			continue;
		}
		
		//获取上次控制时间
		TIME lastXYCtrlTime, lastZCtrlTime;
		get_lastXYCtrlTime(&lastXYCtrlTime);
		get_lastZCtrlTime(&lastZCtrlTime);
		
		if( lastXYCtrlTime.get_pass_time()>1 || lastZCtrlTime.get_pass_time()>1 )
		{	//控制超时
			//强制进入MSafe控制
			ForceMSafeCtrl = true;			
			//打开高度控制器
			Altitude_Control_Enable();
			
			//获取接收机
			Receiver rc;
			getReceiver(&rc);
			
			if( rc.available )
			{	//使用遥控器控制
		
				//复位自动控制计时器
				firstAuto = 0;
				
				//根据5通状态选择定点定高				
				if( rc.data[4] > 60 )
				{
					Position_Control_Enable();
				}
				else if( rc.data[4] < 40 )
				{
					Position_Control_Disable();
				}
				
				bool posCtrlEna;
				is_Position_Control_Enabled(&posCtrlEna);
				
				//油门杆控制垂直速度
				if( in_symmetry_range_mid( rc.data[0] , 50 , 5 ) )
					Position_Control_set_ZLock();
				else
					Position_Control_set_TargetVelocityZ( ( remove_deadband( rc.data[0] - 50.0 , 5.0 ) ) * 6 );
				//偏航杆在中间锁偏航
				//不在中间控制偏航速度
				if( in_symmetry_range_mid( rc.data[1] , 50 , 5 ) )
					Attitude_Control_set_YawLock();
				else
					Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*0.05 );
				//水平控制
				if( posCtrlEna )
				{
					//俯仰横滚杆控水平速度
					if( in_symmetry_range_mid( rc.data[3] , 50 , 5 ) && in_symmetry_range_mid( rc.data[2] , 50 , 5 ) )
						Position_Control_set_XYLock();
					else
					{
						double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, 5.0 );
						double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, 5.0 );
						Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(
							pitch_sitck_d * 25 ,
							-roll_sitck_d * 25 ,
							fabs( roll_sitck_d  )*0.017,
							fabs( pitch_sitck_d )*0.017
						);
					}
				}
				else
				{
					//俯仰横滚杆控俯仰横滚
					Attitude_Control_set_Target_RollPitch( 
						( rc.data[3] - 50 )*0.015,
						( rc.data[2] - 50 )*0.015
					);
				}
			}
			else
			{	//无遥控信号
				//尝试打开水平位置控制器
				Position_Control_Enable();
				
				bool posCtrlEna;
				is_Position_Control_Enabled(&posCtrlEna);
				
				if( posCtrlEna )
				{	//已打开水平位置控制器
					
					if( firstAuto != 31 )
					{	//刚进入失控状态
						//刹车锁位置
						Position_Control_set_XYLock();
						Position_Control_set_ZLock();
						if( firstAuto != 16 )
						{	//初始化等待
							init_NavCmdInf(&navInf);
							firstAuto = 16;
						}
						else
						{	//刹车后等待2秒
							Position_ControlMode alt_mode, pos_mode;
							get_Altitude_ControlMode(&alt_mode);
							get_Position_ControlMode(&pos_mode);
							if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
							{
								if( ++navInf.counter2 >= 2*MSafeRate )
								{
									init_NavCmdInf(&navInf);								
									current_mission_ind = 0;
									firstAuto = 31;
								}
							}
						}
					}
					else
					{	//开始自动飞行（返航）
						switch( current_mission_ind )
						{
							
							case 0:
							{	//判断返航距离是否需要升高																
								vector2<double> homeP;
								if( getHomeLatLon(&homeP) )
								{	//返航至经纬度
									
									//获取最优全球定位传感器信息
									PosSensorHealthInf2 global_inf;
									if( get_OptimalGlobal_XY( &global_inf ) == false )
									{
										goto RtLocal;
									}
									//获取指定经纬度平面坐标
									double x, y;
									map_projection_project( &global_inf.mp, homeP.x, homeP.y, &x, &y );
									x -= global_inf.HOffset.x;
									y -= global_inf.HOffset.y;
									double RtDistanceSq = sq(y - global_inf.PositionENU.y) + sq(x - global_inf.PositionENU.x);
									
									//判断是否大于升高返航距离
									float RtRange[2];
									if( ReadParam( "Sf_GbRtHRange", 0, 0, (uint64_t*)RtRange, 0 ) == PR_OK )
									{
										if( RtDistanceSq > sq(RtRange[0]) )
										{	//升高返航
											float h[2];
											if( ReadParam( "Sf_GbRtHeight", 0, 0, (uint64_t*)h, 0 ) == PR_OK )
												RtHeight = h[0];
											else
												RtHeight = -1;
										}
									}
									else
										RtHeight = -1;
									
									//切换至升高状态
									init_NavCmdInf(&navInf);								
									++current_mission_ind;
								}
								else if( getHomePoint(&homeP) )
								{	//返航至Local坐标
RtLocal:
									vector3<double> position;
									get_Position(&position);
									double RtDistanceSq = sq(homeP.y - position.y) + sq(homeP.x - position.x);
									
									//判断是否大于升高返航距离
									float RtRange[2];
									if( ReadParam( "Sf_LcRtHRange", 0, 0, (uint64_t*)RtRange, 0 ) == PR_OK )
									{
										if( RtDistanceSq > sq(RtRange[0]) )
										{	//升高返航
											float h[2];
											if( ReadParam( "Sf_LcRtHeight", 0, 0, (uint64_t*)h, 0 ) == PR_OK )
												RtHeight = h[0];
											else
												RtHeight = -1;
										}
									}
									else
										RtHeight = -1;
									
									//切换至升高状态
									init_NavCmdInf(&navInf);								
									++current_mission_ind;
								}
								else
								{	//无返航点
									//直接降落
									current_mission_ind = 3;
									break;
								}
								break;
							}
							
							case 1:
							{	//升高到指定对地高度
								double homeZ;
								getHomeLocalZ(&homeZ);
								vector3<double> pos;
								get_Position(&pos);
								if( RtHeight>0 && homeZ+RtHeight<pos.z )
								{
									double params[7];
									params[0] = 0;
									params[1] = 0;
									params[2] = 0;
									params[3] = nan("");
									params[4] = 230;	params[5] = 230;
									params[6] = RtHeight*0.01;
									int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, MSafeRate, MAV_FRAME_GLOBAL_RELATIVE_ALT, params, &navInf );
									if( res != -2 )
									{
										init_NavCmdInf(&navInf);								
										++current_mission_ind;
									}
								}
								else
								{
									init_NavCmdInf(&navInf);								
									++current_mission_ind;
								}							
								break;
							}
							
							case 2:
							{	//回到Home点
								double params[7];
								params[0] = 0;
								params[1] = 0;
								params[2] = 0;
								params[3] = 0;
								params[4] = 0;	params[5] = 0;
								params[6] = 0;
								int16_t res = Process_NavCmd( MAV_CMD_NAV_RETURN_TO_LAUNCH, MSafeRate, MAV_FRAME_GLOBAL_RELATIVE_ALT, params, &navInf );
								if( res != -2 )
								{
									init_NavCmdInf(&navInf);								
									++current_mission_ind;
								}
								break;
							}
							
							default:
							case 3:
							{	//降落
								Position_Control_set_TargetVelocityZ(-50);
								break;
							}
							
						}
					}
				}
				else
				{	//无法打开位置控制器
					//让姿态回复水平
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					//下降
					Position_Control_set_TargetVelocityZ(-50);
					//复位自动控制计时器
					firstAuto = 0;
				}
			}
			
			//已降落关闭控制器并退出MSafe
			bool inFlight;
			get_is_inFlight(&inFlight);
			if( inFlight==false )
			{
				Attitude_Control_Disable();
				ForceMSafeCtrl = false;
			}
		}
		else
		{
			ForceMSafeCtrl = false;
			
			//复位自动控制计时器
			firstAuto = 0;
		}
	}
}

void init_MSafe()
{
	//注册参数
	MSafeCfg initial_cfg;
	initial_cfg.SfRtMode[0] = 1;	//SfRtMode
	initial_cfg.RtSpeed[0] = 500;	//RtSpeed
	initial_cfg.GbRtHRange[0] = 1500;	//GbRtHRange
	initial_cfg.LcRtHRange[0] = 1500;	//LcRtHRange
	initial_cfg.GbRtHeight[0] = 10000;	//GbRtHeight
	initial_cfg.LcRtHeight[0] = 2000;	//LcRtHeight
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT8 ,	//SfRtMode
		MAV_PARAM_TYPE_REAL32 ,	//RtSpeed
		
		MAV_PARAM_TYPE_REAL32 ,	//GbRtHRange
		MAV_PARAM_TYPE_REAL32 ,	//LcRtHRange
		
		MAV_PARAM_TYPE_REAL32 ,	//GbRtHeight		
		MAV_PARAM_TYPE_REAL32 ,	//LcRtHeight
	};
	SName param_names[] = {
		"Sf_SfRtMode" ,	//SfRtMode		
		"Sf_RtSpeed" ,	//RtSpeed
		
		"Sf_GbRtHRange" ,	//GbRtHRange
		"Sf_LcRtHRange" ,	//LcRtHRange
		
		"Sf_GbRtHeight" ,	//GbRtHeight		
		"Sf_LcRtHeight" ,	//LcRtHeight
	};
	ParamGroupRegister( "MSafe", 3, 6, param_types, param_names, (uint64_t*)&initial_cfg );
	
	xTaskCreate( MSafe_Server, "MSafe", 1024, NULL, SysPriority_SafeTask, &MSafeTaskHandle);
}