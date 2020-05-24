#include "M30_AttCtrl.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"

M30_AttCtrl::M30_AttCtrl():Mode_Base( "AttCtrl", 30 )
{
	
}

ModeResult M30_AttCtrl::main_func( void* param1, uint32_t param2 )
{
	setLedMode(LEDMode_Flying1);
	Attitude_Control_Enable();
	uint16_t exit_mode_counter_rs = 0;
	uint16_t exit_mode_counter = 0;
	uint16_t exit_mode_Gcounter = 0;
	while(1)
	{
		os_delay(0.02);
			
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		if( rc.available )
		{
			/*判断退出模式*/
				//获取飞行状态
				bool inFlight;
				get_is_inFlight(&inFlight);
				if( inFlight )
				{
					exit_mode_counter_rs = 400;
					if( exit_mode_counter < exit_mode_counter_rs )
						exit_mode_counter = exit_mode_counter_rs;
				}
				//降落后自动加锁
				if( inFlight==false && rc.data[0]<30 )
				{
					if( ++exit_mode_counter >= 500 )
					{
						Attitude_Control_Disable();
						return MR_OK;
					}
				}
				else
					exit_mode_counter = exit_mode_counter_rs;
				//手势加锁
				if( inFlight==false && (rc.data[0] < 5 && rc.data[1] < 5 && rc.data[2] < 5 && rc.data[3] > 95) )
				{
					if( ++exit_mode_Gcounter >= 50 )
					{
						Attitude_Control_Disable();
						return MR_OK;
					}
				}
				else
					exit_mode_Gcounter = 0;
			/*判断退出模式*/
			
			//油门杆直接控油门
			Attitude_Control_set_Throttle(rc.data[0]);
			//俯仰横滚杆控俯仰横滚
			Attitude_Control_set_Target_RollPitch( ( rc.data[3] - 50 )*0.015 , ( rc.data[2] - 50 )*0.015 );
			//偏航杆在中间锁偏航
			//不在中间控制偏航速度
			if( in_symmetry_range_mid( rc.data[1] , 50 , 5 ) )
				Attitude_Control_set_YawLock();
			else
				Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*0.05 );
		}
		else
		{
			//无遥控信号进入安全模式
			enter_MSafe();		
			/*判断退出模式*/
				bool inFlight;
				get_is_inFlight(&inFlight);
				if( inFlight==false )
				{
					Attitude_Control_Disable();
					return MR_OK;
				}
			/*判断退出模式*/
		}
	}
	return MR_OK;
}
