#include "M35_Auto1.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "NavCmdProcess.hpp"

M35_Auto1::M35_Auto1():Mode_Base( "Auto1", 35 )
{
	
}

ModeResult M35_Auto1::main_func( void* param1, uint32_t param2 )
{
	double freq = 50;
	setLedMode(LEDMode_Flying1);
	Altitude_Control_Enable();
	Position_Control_Enable();
	uint16_t exit_mode_counter_rs = 0;
	uint16_t exit_mode_counter = 0;
	uint16_t exit_mode_Gcounter = 0;
	
	//任务模式
	bool MissionMode = true;
	bool mode_switched = true;
	double lastMissionButtonValue = -1;
	//当前执行任务的序号
	uint16_t mission_ind = 0;
	//任务状态机
	NavCmdInf navInf;
	init_NavCmdInf(&navInf);
	while(1)
	{
		os_delay(0.02);
		
		//获取接收机
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		//获取消息
		bool msg_available;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg, 0 );
		
		//接收机可用
		//进行任务、手动模式切换
		if( rc.available )
		{
			double MissionButtonValue = rc.data[5];
			if( lastMissionButtonValue < 0 )
				lastMissionButtonValue = MissionButtonValue;
			else if( fabs( MissionButtonValue - lastMissionButtonValue ) > 25 )
			{
				MissionMode = !MissionMode;
				mode_switched = true;
				init_NavCmdInf(&navInf);
				lastMissionButtonValue = MissionButtonValue;
			}
		}
		else
			lastMissionButtonValue = -1;
		
		//指令切换任务、手动飞行
		if( msg_available )
		{
			if( msg.cmd == MAV_CMD_NAV_GUIDED_ENABLE )
			{	//进入任务飞行
				if( msg.params[0]==0.5f && MissionMode==false )
				{					
					MissionMode = true;
					mode_switched = true;
					init_NavCmdInf(&navInf);
					if( rc.available )
					{
						double MissionButtonValue = rc.data[5];
						lastMissionButtonValue = MissionButtonValue;
					}
				}
				else if( msg.params[0]!=0.5f && MissionMode==true )
				{
					MissionMode = false;
					mode_switched = true;
					init_NavCmdInf(&navInf);
					if( rc.available )
					{
						double MissionButtonValue = rc.data[5];
						lastMissionButtonValue = MissionButtonValue;
					}
				}
			}
		}
		
		if( MissionMode )
		{	//任务模式
			Position_Control_Enable();
			bool pos_ena;
			is_Position_Control_Enabled(&pos_ena);
			if( pos_ena == false )
			{	//位置控制器无法打开返回手动模式
				MissionMode = false;
				goto Manual_Mode;
			}
			
			if( mode_switched )
			{	//刚切换到任务模式
				//首先刹车等待
				
				if( rc.available )
				{
					bool sticks_in_neutral = 
						in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[3] , 50 , 5 );
					if( !sticks_in_neutral )
					{	//摇杆不在中间返回手动模式
						init_NavCmdInf(&navInf);
						MissionMode = false;
						goto Manual_Mode;
					}
				}
				
				Position_Control_set_XYLock();
				Position_Control_set_ZLock();
				
				//等待刹车完成
				Position_ControlMode alt_mode, pos_mode;
				get_Altitude_ControlMode(&alt_mode);
				get_Position_ControlMode(&pos_mode);
				if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
				{	//刹车完成
					++navInf.counter2;
					//等待1秒再进入任务飞行
					if( navInf.counter2 >= 1*freq )
					{	//进入任务飞行模式
						mode_switched = false;
					}
				}
				else
					navInf.counter2 = 0;
			}
			else
			{	//任务飞行				
				if( rc.available )
				{
					bool sticks_in_neutral = 
						in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[3] , 50 , 5 );
					if( !sticks_in_neutral )
					{	//打杆返回手动模式
						init_NavCmdInf(&navInf);
						MissionMode = false;
						goto Manual_Mode;
					}
				}
				
				//根据mission_ind状态判断当前需要执行什么飞行动作
				switch( mission_ind )
				{
					case 0:
					{	//起飞
						double params[7];
						params[0] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 0;
						params[6] = 0.5;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_TAKEOFF, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( res != -2 )
						{	//起飞完成
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 1:
					{	//飞直线
						double params[7];
						params[0] = 2;
						params[1] = 0;
						params[2] = 0;
						params[3] = nan("");
						params[4] = 0.5;	params[5] = 0;
						params[6] = 0.5;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( res != -2 )
						{	//飞直线完成
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 2:
					{	//飞直线
						double params[7];
						params[0] = 2;
						params[1] = 0;
						params[2] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 0.5;
						params[6] = 0;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( res != -2 )
						{	//飞直线完成
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 3:
					{	//飞直线
						double params[7];
						params[0] = 2;
						params[1] = 0;
						params[2] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 0.5;
						params[6] = 0;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( res != -2 )
						{	//飞直线完成
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 4:
					{	//飞直线
						double params[7];
						params[0] = 2;
						params[1] = 0;
						params[2] = 0;
						params[3] = nan("");
						params[4] = 0;	params[5] = 0.5;
						params[6] = -0.5;
						int16_t res = Process_NavCmd( MAV_CMD_NAV_WAYPOINT, freq, MAV_FRAME_BODY_FLU, params, &navInf );
						if( res != -2 )
						{	//飞直线完成
							init_NavCmdInf(&navInf);
							++mission_ind;
						}
						break;
					}
					
					case 5:
					{	//降落
						Position_Control_set_XYLock();
						Position_Control_set_TargetVelocityZ(-50);
						break;
					}
					
					default:
					{
						MissionMode = false;
						mission_ind = 0;
						goto Manual_Mode;
						break;
					}
				}
			}
		}
		else
		{	//手动飞行模式（包含定高定点控制）
			Manual_Mode:
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
				
				if( rc.data[4] > 60 )
				{
					Position_Control_Enable();
				}
				else if( rc.data[4] < 40 )
				{
					Position_Control_Disable();
				}
					
				//油门杆控制垂直速度
				if( in_symmetry_range_mid( rc.data[0] , 50 , 5 ) )
					Position_Control_set_ZLock();
				else
					Position_Control_set_TargetVelocityZ( ( remove_deadband( rc.data[0] - 50.0 , 5.0 ) ) * 6 );
				
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
				if( pos_ena )
				{
					//俯仰横滚杆控水平速度
					if( in_symmetry_range_mid( rc.data[3] , 50 , 5 ) && in_symmetry_range_mid( rc.data[2] , 50 , 5 ) )
						Position_Control_set_XYLock();
					else
					{
						double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, 5.0 );
						double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, 5.0 );
						Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
							pitch_sitck_d * 25 ,\
							-roll_sitck_d * 25 , \
							fabs( roll_sitck_d  )*0.017, \
							fabs( pitch_sitck_d )*0.017 \
						);
					}
				}
				else
				{
					//补偿风力扰动
					vector3<double> WindDisturbance;
					get_WindDisturbance( &WindDisturbance );
					Quaternion attitude;
					get_Attitude_quat(&attitude);
					double yaw = attitude.getYaw();		
					double sin_Yaw, cos_Yaw;
					fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
					double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
					double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
					//俯仰横滚杆控俯仰横滚
	//				Attitude_Control_set_Target_RollPitch( 
	//					( rc.data[3] - 50 )*0.015 - atan2(-WindDisturbance_Bodyheading_y , GravityAcc ),
	//					( rc.data[2] - 50 )*0.015 - atan2( WindDisturbance_Bodyheading_x , GravityAcc ) 
	//				);
					Attitude_Control_set_Target_RollPitch( 
						( rc.data[3] - 50 )*0.015,
						( rc.data[2] - 50 )*0.015
					);
				}
				
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
	}
	return MR_OK;
}