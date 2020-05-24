
#include "Basic.hpp"
#include "ctrl_Main.hpp"
#include "ctrl_Attitude.hpp"
#include "ctrl_Position.hpp"
#include "MeasurementSystem.hpp"
#include "ControlSystem.hpp"
#include "drv_ADC.hpp"
#include "Parameters.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

/*控制系统互斥锁*/
	static SemaphoreHandle_t CtrlMutex = xSemaphoreCreateRecursiveMutex();
	bool LockCtrl( double TIMEOUT )
	{
		TickType_t TIMTOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMTOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMTOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTakeRecursive( CtrlMutex, TIMTOUT_Ticks ) == pdTRUE )
			return true;
		else
			return false;
	}
	void UnlockCtrl()
	{
		xSemaphoreGiveRecursive(CtrlMutex);
	}
/*控制系统互斥锁*/
	
/*控制系统安全时间*/
	TIME last_XYCtrlTime;
	TIME last_ZCtrlTime;
	//获取控制器上次控制时间
	//太久不进行控制将进入MSafe模式
	bool get_lastXYCtrlTime( TIME* t, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*t = last_XYCtrlTime;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_lastZCtrlTime( TIME* t, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*t = last_ZCtrlTime;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	//将控制器上次控制时间设为不可用
	//强制进入MSafe模式
	//MSafe模式下无法关闭位置控制器
	//同时作出XYZ位置控制可退出MSafe
	//（水平控制不可用时控制角度）
	bool enter_MSafe( double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			last_XYCtrlTime.set_invalid();
			last_ZCtrlTime.set_invalid();
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	//强制Safe控制
	bool ForceMSafeCtrl = false;
	
	//获取是否进入了MSafe模式
	bool is_MSafeCtrl()
	{
		return ForceMSafeCtrl;
	}
/*控制系统安全时间*/

/*电压电流功率数据*/
	//滤波电压（V）
	static float MainBatteryVoltage_filted = 0;
	//总使用功耗（W*h）
	static float MainBatteryPowerUsage = 0;
	//滤波功率（W）
	static float MainBatteryPower_filted = 0;
	
	float get_MainBatteryVoltage() { return Get_MainBaterry_Voltage(); }
	float get_MainBatteryVoltage_filted() { return MainBatteryVoltage_filted; }
	float get_MainBatteryPowerUsage() { return MainBatteryPowerUsage; }
	float get_MainBatteryPower_filted() { return MainBatteryPower_filted; }
	float get_MainBatteryCurrent() { return Get_MainBaterry_Current(); }
	float get_CPUTemerature() { return Get_Temperature(); }
	void get_MainBatteryInf( float* Volt, float* Current, float* PowerUsage, float* Power_filted, float* RMPercent )
	{		
		float volt = Get_MainBaterry_Voltage();
		if( Volt != 0 )
			*Volt = volt;
		if( *Current != 0 )
			*Current = Get_MainBaterry_Current();
		if( *PowerUsage != 0 )
			*PowerUsage = MainBatteryPowerUsage;
		if( *Power_filted != 0 )
			*Power_filted = MainBatteryPower_filted;
		if( *RMPercent != 0 )
		{
			BatteryCfg cfg;
			if( ReadParamGroup( "Battery", (uint64_t*)&cfg, 0 ) == PR_OK )
			{
				if( volt > cfg.STVoltage[0] + cfg.VoltP10[0] )
				{
					*RMPercent = 100;
					return;
				}
				for( int8_t i = 10; i >= 1 ; --i )
				{
					float P1 = cfg.STVoltage[0] + (&(cfg.VoltP0[0]))[(i-1)*2];					
					if( volt > P1 )
					{
						float P2 = cfg.STVoltage[0] + (&(cfg.VoltP0[0]))[(i-0)*2];
						*RMPercent = (i-1)*10 + 10*( volt - P1 ) / ( P2 - P1 );
						return;
					}
				}
				*RMPercent = 0;
			}
			else
				*RMPercent = 100;
		}
	}
/*电压电流功率数据*/
	
static void ControlSystem_Task(void* pvParameters)
{
	//等待驱动初始化完成
	while( getInitializationCompleted() == false )
		os_delay(0.1);
	
	//等待姿态解算系统准备完成
	while( get_Attitude_MSStatus() != MS_Ready )
		os_delay(0.1);
	
	//等待位置解算系统准备完成
	while( get_Altitude_MSStatus() != MS_Ready )
		os_delay(0.1);
	
	//设置初始电压
	uint16_t VoltMeas_counter = 0;
	MainBatteryVoltage_filted = Get_MainBaterry_Voltage();
	//准确周期延时
	static TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//400hz
		vTaskDelayUntil( &xLastWakeTime, 2 );
		
		if( LockCtrl(0.02) )
		{
			ctrl_Position();
			ctrl_Attitude();
			
			UnlockCtrl();
		}
		
		/*电压测量滤波*/
			if( ++VoltMeas_counter >= 0.5*CtrlRateHz )
			{
				VoltMeas_counter = 0;
				
				//电压滤波
				float MBVoltage = Get_MainBaterry_Voltage();
				float MBCurrent = Get_MainBaterry_Current();
				MainBatteryVoltage_filted += 0.1f*0.5f * ( MBVoltage - MainBatteryVoltage_filted );
				//功率滤波
				MainBatteryPower_filted += 0.1f*0.5f * ( MBVoltage*MBCurrent - MainBatteryPower_filted );
				
				//求使用功耗（W*h）
				MainBatteryPowerUsage += (0.5f / 60.0f) * MBVoltage*MBCurrent;
			}
		/*电压测量滤波*/
	}
}

void init_ControlSystem()
{
	init_Ctrl_Attitude();
	init_Ctrl_Position();
	xTaskCreate( ControlSystem_Task, "ControlSystem", 4096, NULL, SysPriority_ControlSystem, NULL);
	
	/*注册电池参数*/
		BatteryCfg initial_cfg;
		initial_cfg.STVoltage[0] = 11.6;
		initial_cfg.VoltMKp[0] = 21;
		initial_cfg.CurrentMKp[0] = 0.02;
		initial_cfg.Capacity[0] = 300;
		initial_cfg.VoltP0[0] = -1.0;
		initial_cfg.VoltP1[0] = -0.6;
		initial_cfg.VoltP2[0] = -0.4;
		initial_cfg.VoltP3[0] = -0.2;
		initial_cfg.VoltP4[0] = -0.0;
		initial_cfg.VoltP5[0] = +0.1;
		initial_cfg.VoltP6[0] = +0.2;
		initial_cfg.VoltP7[0] = +0.3;
		initial_cfg.VoltP8[0] = +0.4;
		initial_cfg.VoltP9[0] = +0.5;
		initial_cfg.VoltP10[0] = +0.6;
	
		MAV_PARAM_TYPE param_types[] = {
			MAV_PARAM_TYPE_REAL32 ,	//STVoltage
			MAV_PARAM_TYPE_REAL32 ,	//VoltMKp
			MAV_PARAM_TYPE_REAL32 ,	//CurrentMKp
			MAV_PARAM_TYPE_REAL32 ,	//Capacity
			MAV_PARAM_TYPE_REAL32 ,	//VoltP0
			MAV_PARAM_TYPE_REAL32 ,	//VoltP1
			MAV_PARAM_TYPE_REAL32 ,	//VoltP2
			MAV_PARAM_TYPE_REAL32 ,	//VoltP3
			MAV_PARAM_TYPE_REAL32 ,	//VoltP4
			MAV_PARAM_TYPE_REAL32 ,	//VoltP5
			MAV_PARAM_TYPE_REAL32 ,	//VoltP6
			MAV_PARAM_TYPE_REAL32 ,	//VoltP7
			MAV_PARAM_TYPE_REAL32 ,	//VoltP8
			MAV_PARAM_TYPE_REAL32 ,	//VoltP9
			MAV_PARAM_TYPE_REAL32 ,	//VoltP10
		};
		SName param_names[] = {
			"Bat_STVoltage" ,	//UAV Type
			"Bat_VoltMKp" ,	//VoltMKp
			"Bat_CurrentMKp" ,	//CurrentMKp
			"Bat_Capacity" ,	//Capacity
			"Bat_VoltP0" ,	//VoltP0
			"Bat_VoltP1" ,	//VoltP1
			"Bat_VoltP2" ,	//VoltP2
			"Bat_VoltP3" ,	//VoltP3
			"Bat_VoltP4" ,	//VoltP4
			"Bat_VoltP5" ,	//VoltP5
			"Bat_VoltP6" ,	//VoltP6
			"Bat_VoltP7" ,	//VoltP7
			"Bat_VoltP8" ,	//VoltP8
			"Bat_VoltP9" ,	//VoltP9
			"Bat_VoltP10" ,	//VoltP10
		};
		ParamGroupRegister( "Battery", 1, 15, param_types, param_names, (uint64_t*)&initial_cfg );
	/*注册电池参数*/
}