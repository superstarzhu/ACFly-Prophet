#include "ctrl_Attitude.hpp"
#include "ControlSystem.hpp"
#include "ctrl_Main.hpp"
#include "Parameters.hpp"
#include "MeasurementSystem.hpp"
#include "TD4.hpp"
#include "TD3_3D.hpp"
#include "ESO_AngularRate.hpp"
#include "ESO_h.hpp"
#include "Filters_LP.hpp"
#include "drv_PWMOut.hpp"
#include "Receiver.hpp"

/*参数*/
	//飞行器类型
	enum UAVType
	{
		UAVType_Rotor4_X = 10 ,	//四旋翼X型
		UAVType_Rotor6_X = 11 ,	//六旋翼X型
		UAVType_Rotor8_X = 12 ,	//八旋翼X型
		
		UAVType_Rotor4_C = 15 ,	//四旋翼十字型
		
		UAVType_Rotor42_C = 20 ,	//四旋翼Double十字型
		
		UAVType_Rotor6_S1 = 32 ,	//六旋翼异构
	};
	//控制参数
	struct AttCtrlCfg
	{
		uint64_t UAVType;	//机型类型
		float STThrottle[2];	//起转油门
		float NonlinearFactor[2];	//电机非线性参数
		float FullThrRatio[2];	//满油门比例
		float T[2];	//惯性时间T
		float b[6];	//RPY增益b
		float TD4_P1[6];	//RPY前馈TD4滤波器P1
		float TD4_P2[6];	//RPY前馈TD4滤波器P2
		float TD4_P3[6];	//RPY前馈TD4滤波器P3
		float TD4_P4[6];	//RPY前馈TD4滤波器P4
		float P1[6];	//反馈增益P1
		float P2[6];	//反馈增益P2
		float P3[6];	//反馈增益P3
		float P4[6];	//反馈增益P4
	}__PACKED;

	//参数
	static UAVType uav_type;	//机型类型
	static double STThrottle;	//起转油门
	static double NonlinearFactor;	//电机非线性参数
	static double FullThrRatio = 0;	//满油门比例
	static double b[3];	//RPY增益b
	static vector3<double> P1;	//反馈增益P1
	static vector3<double> P2;	//反馈增益P2
	static vector3<double> P3;	//反馈增益P3
	static vector3<double> P4;	//反馈增益P4
/*参数*/
	
/*内部接口*/
	double get_STThrottle()
	{
		return STThrottle;
	}
/*内部接口*/
	
/*起飞地点*/
	static bool HomeLatLonAvailable;
	static bool HomeAvailable;
	static vector2<double> HomeLatLon;
	static vector2<double> HomePoint;
	static double HomeLocalZ = 0;
	bool getHomeLocalZ( double* home, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*home = HomeLocalZ;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool getHomePoint( vector2<double>* home, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			bool available = HomeAvailable;
			if(available)
				*home = HomePoint;
			UnlockCtrl();
			if( available )
				return true;
			else
				return false;
		}
		return false;
	}
	bool getHomeLatLon( vector2<double>* home, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			bool available = HomeLatLonAvailable;
			if(available)
				*home = HomeLatLon;
			UnlockCtrl();
			if( available )
				return true;
			else
				return false;
		}
		return false;
	}
/*起飞地点*/
	
/*状态观测器*/
	//高度ESO
	static ESO_h ESO_height;
	static double hover_throttle = 0;
	static double WindDisturbance_x = 0;
	static double WindDisturbance_y = 0;
	static bool inFlight = false;
	static inline void update_output_throttle( double throttle, double h )
	{
		Quaternion quat;
		get_Airframe_quat( &quat, 0.1 );
		vector3<double> AccENU;
		get_AccelerationENU_Ctrl( &AccENU, 0.1 );
		double output_minimum_throttle = STThrottle;
		double lean_cosin = quat.get_lean_angle_cosin();
		
		//观测悬停油门
		double r_throttle = throttle - output_minimum_throttle;
		if( r_throttle < 0 )
			r_throttle = 0;
		r_throttle *= lean_cosin;
		ESO_height.update_u(r_throttle);
		hover_throttle = ESO_height.run( AccENU.z, h ) + output_minimum_throttle;
		
		//更新飞行状态
		static uint16_t onGround_counter = 0;
		if( inFlight == false )
		{
			onGround_counter = 0;
			if( AccENU.z > 20 && throttle > output_minimum_throttle + 5 )
				inFlight = true;
		}
		else
		{
			if( hover_throttle < output_minimum_throttle + 2 )
			{
				if( ++onGround_counter >= 600 )
					inFlight = false;
			}
			else
				onGround_counter = 0;
		}
		
		//观测水平分力
		if( inFlight )
		{
			Quaternion quat;
			get_history_AirframeQuatY( &quat, 0.42/10, 0.1 );
			vector3<double> active_force_xy_vec = quat.rotate_axis_z();
			if( lean_cosin < 0.3f )
				lean_cosin = 0.3f;
			active_force_xy_vec = active_force_xy_vec *( ( AccENU.z + GravityAcc ) / lean_cosin );
			vector3<double> WindDisturbance_xy;
			WindDisturbance_xy.x = AccENU.x - active_force_xy_vec.x;
			WindDisturbance_xy.y = AccENU.y - active_force_xy_vec.y;
			
			double lp_factor = 2 * Pi * (1.0/CtrlRateHz) * 0.5;
			WindDisturbance_x += lp_factor * ( WindDisturbance_xy.x - WindDisturbance_x );
			WindDisturbance_y += lp_factor * ( WindDisturbance_xy.y - WindDisturbance_y );
		}
		else
			WindDisturbance_x = WindDisturbance_y = 0;
		
		//更新Home点位置
		if( inFlight == false )
		{
			vector3<double> position;
			get_Position(&position);
			HomeLocalZ = position.z;
			
			PosSensorHealthInf2 posInf;
			if( get_Health_XY(&posInf) )
			{
				HomeAvailable = true;
				HomePoint.x = posInf.PositionENU.x;
				HomePoint.y = posInf.PositionENU.y;
			}
			else
				HomeAvailable = false;
			
			if( get_OptimalGlobal_XY(&posInf) )
			{
				HomeLatLonAvailable = true;
				map_projection_reproject( &posInf.mp, 
					posInf.PositionENU.x+posInf.HOffset.x, 
					posInf.PositionENU.y+posInf.HOffset.y,
					&HomeLatLon.x, &HomeLatLon.y );
			}			
		}
		else if( get_Position_MSStatus()!= MS_Ready )
			HomeAvailable = false;
	}
/*状态观测器*/	

/*观测器接口*/
	bool get_hover_throttle( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = hover_throttle;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_is_inFlight( bool* result, double TIMEOUT )
	{
		*result = inFlight;
		return true;
	}
	bool get_WindDisturbance( vector3<double>* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = vector3<double>( WindDisturbance_x, WindDisturbance_y, 0 );;
			UnlockCtrl();
			return true;
		}
		return false;		
	}
/*观测器接口*/
	
/*控制接口*/
	//期望TD4滤波器
	static TD3_2DSL Target_tracker_RP;
	static TD4 Target_tracker[3];
	//姿态ESO
	static ESO_AngularRate ESO[3];
	//扰动滤波器
	static Filter_Butter2_LP disturbance_filter[3];
	
	//姿态控制模式
	static bool Attitude_Control_Enabled = false;
	static Attitude_ControlMode RollPitch_ControlMode = Attitude_ControlMode_Angle;
	static Attitude_ControlMode Yaw_ControlMode = Attitude_ControlMode_Angle;

	static double throttle = 0;
	static double target_Roll;
	static double target_Pitch;
	static double target_Yaw;
	static vector3<double> target_AngularRate;
	
	bool is_Attitude_Control_Enabled( bool* enabled, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*enabled = Attitude_Control_Enabled;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_Enable( double TIMEOUT )
	{
		if( get_Attitude_MSStatus() != MS_Ready )
			return false;
		
		Quaternion quat;
		if( get_Airframe_quat( &quat, TIMEOUT ) == false )
			return false;
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == true )
			{	//控制器已打开
				UnlockCtrl();
				return false;
			}
			
			//读参数
			AttCtrlCfg cfg;
			if( ReadParamGroup( "AttCtrl", (uint64_t*)&cfg, 0, TIMEOUT ) != PR_OK )
			{
				UnlockCtrl();
				return false;
			}
			
			Attitude_Control_Enabled = true;
			target_Yaw = quat.getYaw();
			target_Roll = target_Pitch = 0;
			RollPitch_ControlMode = Attitude_ControlMode_Angle;
			Yaw_ControlMode = Attitude_ControlMode_Angle;
			
			/*初始化*/
				//初始化飞行器类型
				uav_type = (UAVType)cfg.UAVType;
			
				//初始化电机参数
				STThrottle = cfg.STThrottle[0];
				NonlinearFactor = cfg.NonlinearFactor[0];
				FullThrRatio = cfg.FullThrRatio[0];
				
				//读取电池电压
				float BatVoltage = get_MainBatteryVoltage_filted();
				//计算增益修正系数
				double b_scale = 1.0;
				if( BatVoltage > 7 )
				{
					float STVoltage[2];
					if( ReadParam("Bat_STVoltage", 0, 0, (uint64_t*)STVoltage, 0 ) == PR_OK )
						b_scale = BatVoltage / STVoltage[0];
				}
			
				//初始化姿态ESO
				b[0] = cfg.b[0];	b[1] = cfg.b[2];	b[2] = cfg.b[4];	
				ESO[0].init( cfg.T[0], b[0]*b_scale, 0.8, 12, 500, 100000 );
				ESO[1].init( cfg.T[0], b[1]*b_scale, 0.8, 12, 500, 100000 );
				ESO[2].init( 1.0/CtrlRateHz, b[2]*b_scale, 0.7, 12, 500, 100000 );
				
				//初始化高度ESO
				ESO_height.init( cfg.T[0], 0.05 );
			
				//初始化期望TD4滤波器
				Target_tracker_RP.P1=15;
				Target_tracker_RP.P2=15;
				Target_tracker_RP.P3=cfg.TD4_P3[0];
				Target_tracker_RP.r2=degree2rad(500.0);
				Target_tracker_RP.r3=degree2rad(5000.0);
				Target_tracker_RP.r4=degree2rad(100000.0);
				Target_tracker[0].P1=cfg.TD4_P1[0];	Target_tracker[1].P1=cfg.TD4_P1[2];	Target_tracker[2].P1=cfg.TD4_P1[4];
				Target_tracker[0].P2=cfg.TD4_P2[0];	Target_tracker[1].P2=cfg.TD4_P2[2];	Target_tracker[2].P2=cfg.TD4_P2[4];
				Target_tracker[0].P3=cfg.TD4_P3[0];	Target_tracker[1].P3=cfg.TD4_P3[2];	Target_tracker[2].P3=cfg.TD4_P3[4];
				Target_tracker[0].P4=cfg.TD4_P4[0];	Target_tracker[1].P4=cfg.TD4_P4[2];	Target_tracker[2].P4=cfg.TD4_P4[4];
			
				//初始化反馈增益
				P1[0] = cfg.P1[0];	P1[1] = cfg.P1[2];	P1[2] = cfg.P1[4];
				P2[0] = cfg.P2[0];	P2[1] = cfg.P2[2];	P2[2] = cfg.P2[4];
				P3[0] = cfg.P3[0];	P3[1] = cfg.P3[2];	P3[2] = cfg.P3[4];
				P4[0] = cfg.P4[0];	P4[1] = cfg.P4[2];	P4[2] = cfg.P4[4];
			/*初始化*/
			
			//更新控制时间
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if(!isMSafe)
				last_ZCtrlTime = last_XYCtrlTime = TIME::now();
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_Disable( double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			Altitude_Control_Disable();
			Position_Control_Disable();
			Attitude_Control_Enabled = false;			
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	bool get_Target_Throttle( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = throttle;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Throttle( double thr, double TIMEOUT )
	{
		if( isnan(thr) || isinf(thr) )
			return false;
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			bool alt_enabled;
			is_Altitude_Control_Enabled(&alt_enabled);
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if( !isMSafe && alt_enabled==false && ForceMSafeCtrl )
			{	//屏蔽用户控制
				last_ZCtrlTime = TIME::now();
				UnlockCtrl();
				return false;
			}
			
			throttle = thr;
			
			//更新控制时间			
			if(!isMSafe && alt_enabled==false)
				last_ZCtrlTime = TIME::now();
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	bool Attitude_Control_get_Target_RollPitch( double* Roll, double* Pitch, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			
			*Roll = target_Roll;
			*Pitch = target_Pitch;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Target_RollPitch( double Roll, double Pitch, double TIMEOUT )
	{
		if( isnan(Roll) || isinf(Roll) || isnan(Pitch) || isinf(Pitch) )
			return false;
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			bool pos_enabled;
			is_Position_Control_Enabled(&pos_enabled);
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if( !isMSafe && pos_enabled==false && ForceMSafeCtrl )
			{	//屏蔽用户控制
				last_ZCtrlTime = TIME::now();
				UnlockCtrl();
				return false;
			}
			
			double angle = safe_sqrt( Roll*Roll + Pitch*Pitch );
			if( angle > 0.8 )
			{
				double scale = 0.8 / angle;
				Roll *= scale;
				Pitch *= scale;
			}		
			target_Roll = Roll;
			target_Pitch = Pitch;
			RollPitch_ControlMode = Attitude_ControlMode_Angle;
			
			//更新控制时间
			if(!isMSafe && pos_enabled==false)
				last_XYCtrlTime = TIME::now();
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	bool Attitude_Control_set_Target_Yaw( double Yaw, double TIMEOUT )
	{		
		if( isnan(Yaw) || isinf(Yaw) )
			return false;
		
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;
		
		Quaternion quat, quatY;
		get_Airframe_quat(&quat);
		get_AirframeY_quat(&quatY);
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			if( Yaw_ControlMode != Attitude_ControlMode_Angle )
			{
				Target_tracker[2].x1 = quat.getYaw();
				Yaw_ControlMode = Attitude_ControlMode_Angle;
			}
			double yaw_err = Mod( Yaw - quatY.getYaw(), 2*Pi );
			if(yaw_err > Pi)
				yaw_err -= 2*Pi;
			while(yaw_err < -Pi)
				yaw_err += 2*Pi;
			target_Yaw = Target_tracker[2].x1 + yaw_err;
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Target_YawRelative( double Yaw, double TIMEOUT )
	{		
		if( isnan(Yaw) || isinf(Yaw) )
			return false;
		
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;
		
		Quaternion quat;
		get_Airframe_quat(&quat);
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			double currentYaw = quat.getYaw();
			if( Yaw_ControlMode != Attitude_ControlMode_Angle )
			{
				Target_tracker[2].x1 = currentYaw;
				Yaw_ControlMode = Attitude_ControlMode_Angle;
			}
			target_Yaw = Target_tracker[2].x1 + Yaw;
		
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Target_YawRate( double YawRate, double TIMEOUT )
	{
		if( isnan(YawRate) || isinf(YawRate) )
			return false;
		
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;
		
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			target_AngularRate.z = YawRate;
			Yaw_ControlMode = Attitude_ControlMode_AngularRate;
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_YawLock( double TIMEOUT )
	{
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			if( Yaw_ControlMode == Attitude_ControlMode_AngularRate )
				Yaw_ControlMode = Attitude_ControlMode_Locking;
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
/*控制接口*/

//四旋翼X模式
static void ctrl_Attitude_MultiRotor_X4_PWM( double outRoll , double outPitch , double outYaw );
//六旋翼X模式
static void ctrl_Attitude_MultiRotor_X6_PWM( double outRoll , double outPitch , double outYaw );
//六旋翼异构
/*
	o--o
	o--o
	o--o
*/
static void ctrl_Attitude_MultiRotor_S6_1_PWM( double outRoll , double outPitch , double outYaw );
//八旋翼X模式
static void ctrl_Attitude_MultiRotor_X8_PWM( double outRoll , double outPitch , double outYaw );
	
void ctrl_Attitude()
{
	if( FullThrRatio < 0.01 )
	{
		//读参数
		AttCtrlCfg cfg;
		PR_RESULT res= ReadParamGroup( "AttCtrl", (uint64_t*)&cfg, 0, 0.1 );
		
		/*初始化*/
			//初始化飞行器类型
			uav_type = (UAVType)cfg.UAVType;
		
			//初始化电机参数
			STThrottle = cfg.STThrottle[0];
			NonlinearFactor = cfg.NonlinearFactor[0];
			FullThrRatio = cfg.FullThrRatio[0];
		
			//读取电池电压
			float BatVoltage = get_MainBatteryVoltage_filted();
			//计算增益修正系数
			double b_scale = 1.0;
			if( BatVoltage > 7 )
			{
				float STVoltage[2];
				if( ReadParam("Bat_STVoltage", 0, 0, (uint64_t*)STVoltage, 0 ) == PR_OK )
					b_scale = BatVoltage / STVoltage[0];
			}
			
			//初始化姿态ESO
			b[0] = cfg.b[0];	b[1] = cfg.b[2];	b[2] = cfg.b[4];	
			ESO[0].init( cfg.T[0], b[0]*b_scale, 0.6, 7, 500, 5000 );
			ESO[1].init( cfg.T[0], b[1]*b_scale, 0.6, 7, 500, 5000 );
			ESO[2].init( 1.0/CtrlRateHz, b[2]*b_scale, 0.6, 7, 500, 5000 );
			
			//初始化高度ESO
			ESO_height.init( cfg.T[0], 0.05 );
		
			//初始化期望TD4滤波器
			Target_tracker_RP.P1=cfg.TD4_P1[0];
			Target_tracker_RP.P2=cfg.TD4_P2[0];
			Target_tracker_RP.P3=cfg.TD4_P3[0];
			Target_tracker_RP.r2=degree2rad(1000.0);
			Target_tracker_RP.r3=degree2rad(10000.0);
			Target_tracker_RP.r4=degree2rad(100000.0);
			Target_tracker[0].P1=cfg.TD4_P1[0];	Target_tracker[1].P1=cfg.TD4_P1[2];	Target_tracker[2].P1=cfg.TD4_P1[4];
			Target_tracker[0].P2=cfg.TD4_P2[0];	Target_tracker[1].P2=cfg.TD4_P2[2];	Target_tracker[2].P2=cfg.TD4_P2[4];
			Target_tracker[0].P3=cfg.TD4_P3[0];	Target_tracker[1].P3=cfg.TD4_P3[2];	Target_tracker[2].P3=cfg.TD4_P3[4];
			Target_tracker[0].P4=cfg.TD4_P4[0];	Target_tracker[1].P4=cfg.TD4_P4[2];	Target_tracker[2].P4=cfg.TD4_P4[4];
		
			//初始化反馈增益
			P1[0] = cfg.P1[0];	P1[1] = cfg.P1[2];	P1[2] = cfg.P1[4];
			P2[0] = cfg.P2[0];	P2[1] = cfg.P2[2];	P2[2] = cfg.P2[4];
			P3[0] = cfg.P3[0];	P3[1] = cfg.P3[2];	P3[2] = cfg.P3[4];
			P4[0] = cfg.P4[0];	P4[1] = cfg.P4[2];	P4[2] = cfg.P4[4];
		/*初始化*/
	}
	
	if( Attitude_Control_Enabled == false )
	{
		update_output_throttle( 0 , 1.0 / CtrlRateHz );
		PWM_PullDownAll();
		return;
	}
	
	//Debug保护
	//油门低于起转油门拉低所有输出
	Receiver rc;
	getReceiver(&rc);
	if( throttle < STThrottle - 0.1 || ( rc.available && rc.data[0] < STThrottle - 0.1 ) )
	{
		update_output_throttle( 0 , 1.0 / CtrlRateHz );
		PWM_PullDownAll();
		return;
	}
	
	
//	//根据电池电压调整控制对象增益
//	float BatV = getBatteryVoltage();
//	float VST = Cfg_get_BatSTVoltage();
//	if( BatV > 7 && VST > 7 )
//	{
//		float scale = BatV / VST;
//		ESO[0].b = Cfg_get_RPYCtrl_b(0) * scale;
//		ESO[1].b = Cfg_get_RPYCtrl_b(1) * scale;
//		ESO[2].b = Cfg_get_RPYCtrl_b(2) * scale;
//	}
//	else
//	{
//		ESO[0].b = Cfg_get_RPYCtrl_b(0);
//		ESO[1].b = Cfg_get_RPYCtrl_b(1);
//		ESO[2].b = Cfg_get_RPYCtrl_b(2);
//	}
	
	//读取电池电压
	float BatVoltage = get_MainBatteryVoltage_filted();
	//计算增益修正系数
	double b_scale = 1.0;
	if( BatVoltage > 7 )
	{
		float STVoltage[2];
		if( ReadParam("Bat_STVoltage", 0, 0, (uint64_t*)STVoltage, 0 ) == PR_OK )
			b_scale = BatVoltage / STVoltage[0];
	}
	//根据电池电压调整控制对象增益
	ESO[0].b = b[0] * b_scale;
	ESO[1].b = b[1] * b_scale;
	ESO[2].b = b[2] * b_scale;
	
	Quaternion AirframeQuat;
	get_Airframe_quat( &AirframeQuat, 0.1 );
	
	//获取控制参数
	double Ps = P1.x;
	double PsY = P1.z;
	
	//目标Roll Pitch四元数
	Quaternion target_quat_PR;			
	//目标角速度
	vector3<double> target_angular_velocity;

	//获取当前四元数的Pitch Roll分量四元数
	double Yaw = AirframeQuat.getYaw();
	double half_sinYaw, half_cosYaw;
	fast_sin_cos( 0.5*Yaw, &half_sinYaw, &half_cosYaw );
	Quaternion YawQuat(
		half_cosYaw ,
		0 ,
		0 ,
		half_sinYaw
	);
	YawQuat.conjugate();
	Quaternion_Ef current_quat_PR = Quaternion_Ef( YawQuat*AirframeQuat );
	
	//计算旋转矩阵
	current_quat_PR.conjugate();				
	double Rotation_Matrix[3][3];	//反向旋转
	current_quat_PR.get_rotation_matrix(Rotation_Matrix);
	current_quat_PR.conjugate();	
	double Rotation_Matrix_P[3][3]; //正向旋转
	current_quat_PR.get_rotation_matrix(Rotation_Matrix_P);
	
	//运行扩张状态观测器得到估计角速度、角加速度
	
	vector3<double> AngularRateCtrl;
	get_AngularRate_Ctrl( &AngularRateCtrl, 0.1 );
	ESO[0].run( AngularRateCtrl.x , 1.0f / CtrlRateHz );
	ESO[1].run( AngularRateCtrl.y , 1.0f / CtrlRateHz );
	ESO[2].run( AngularRateCtrl.z , 1.0f / CtrlRateHz );
	vector3<double> angular_rate_ESO(
		ESO[0].get_EsAngularRate() ,
		ESO[1].get_EsAngularRate() ,
		ESO[2].get_EsAngularRate()
	);
//	vector3<double> angular_rate_ESO(
//		AngularRateCtrl.x ,
//		AngularRateCtrl.y ,
//		AngularRateCtrl.z
//	);
	vector3<double> angular_acceleration_ESO(
		ESO[0].get_EsAngularAcceleration() ,
		ESO[1].get_EsAngularAcceleration() ,
		ESO[2].get_EsAngularAcceleration()
	);
	
	//计算ENU坐标系下的角速度、角加速度
	vector3<double> angular_rate_ENU;
	angular_rate_ENU.x = Rotation_Matrix_P[0][0]*angular_rate_ESO.x + Rotation_Matrix_P[0][1]*angular_rate_ESO.y + Rotation_Matrix_P[0][2]*angular_rate_ESO.z;
	angular_rate_ENU.y = Rotation_Matrix_P[1][0]*angular_rate_ESO.x + Rotation_Matrix_P[1][1]*angular_rate_ESO.y + Rotation_Matrix_P[1][2]*angular_rate_ESO.z;
	angular_rate_ENU.z = Rotation_Matrix_P[2][0]*angular_rate_ESO.x + Rotation_Matrix_P[2][1]*angular_rate_ESO.y + Rotation_Matrix_P[2][2]*angular_rate_ESO.z;
	vector3<double> angular_acceleration_ENU;
	angular_acceleration_ENU.x = Rotation_Matrix_P[0][0]*angular_acceleration_ESO.x + Rotation_Matrix_P[0][1]*angular_acceleration_ESO.y + Rotation_Matrix_P[0][2]*angular_acceleration_ESO.z;
	angular_acceleration_ENU.y = Rotation_Matrix_P[1][0]*angular_acceleration_ESO.x + Rotation_Matrix_P[1][1]*angular_acceleration_ESO.y + Rotation_Matrix_P[1][2]*angular_acceleration_ESO.z;
	angular_acceleration_ENU.z = Rotation_Matrix_P[2][0]*angular_acceleration_ESO.x + Rotation_Matrix_P[2][1]*angular_acceleration_ESO.y + Rotation_Matrix_P[2][2]*angular_acceleration_ESO.z;
	
	//由Roll Pitch控制模式
	//计算Roll Pitch目标角速度（ENU系）
	vector3<double> target_angular_rate_RP;
	switch( RollPitch_ControlMode )
	{
		default:
		case Attitude_ControlMode_Angle:
		{
			//TD4滤目标角度
			Target_tracker_RP.track3( vector2<double>(target_Roll,target_Pitch), 1.0 / CtrlRateHz );
			
			//使用目标角度构造目标四元数
			//calculate target quat Q1
			//      front
			//       x
			//       ^
			//       |
			// y < --O
			double half_sinR, half_cosR;
			fast_sin_cos( 0.5*Target_tracker_RP.x1.x, &half_sinR, &half_cosR );
			double half_sinP, half_cosP;
			fast_sin_cos( 0.5*Target_tracker_RP.x1.y, &half_sinP, &half_cosP );
			target_quat_PR = Quaternion( 
				half_cosR*half_cosP ,
				half_cosP*half_sinR ,
				half_cosR*half_sinP ,
				-half_sinR*half_sinP
			);
			
			//计算误差四元数Q
			//Q*Q1=Qt  Q1为当前机体四元数，Qt为目标四元数
			//Q=Qt*inv(Q1)
			Quaternion current_quat_conj = current_quat_PR;	current_quat_conj.conjugate();
			vector3<double> PR_rotation = ( target_quat_PR * current_quat_conj ).get_Rotation_vec();
			vector3<double> feed_foward_ratePR = { Target_tracker_RP.x2.x, Target_tracker_RP.x2.y , 0 };
			target_angular_rate_RP = ( PR_rotation * Ps ) + feed_foward_ratePR;
			break;
		}
	}
	
	double target_angular_rate_Y;
	switch(Yaw_ControlMode)
	{
		case Attitude_ControlMode_Angle:
		{
			//TD4滤目标角度
			Target_tracker[2].r2n = Target_tracker[2].r2p = 1.0;
			Target_tracker[2].track4( target_Yaw , 1.0f / CtrlRateHz );
			
			//角度误差化为-180 - +180
			double angle_error = Target_tracker[2].x1 - Yaw;
			while( angle_error < -Pi )
				angle_error+=2*Pi;
			while( angle_error > Pi )
				angle_error-=2*Pi;

			//求目标角速度
			target_angular_rate_Y = angle_error * Ps + Target_tracker[2].x2;
			target_angular_rate_Y = constrain( target_angular_rate_Y , 2.5 );
			break;
		}
		case Attitude_ControlMode_AngularRate:
		{
			Target_tracker[2].track3( target_AngularRate.z , 1.0 / CtrlRateHz );
			target_angular_rate_Y = Target_tracker[2].x2;
			break;
		}
		case Attitude_ControlMode_Locking:
		{
			Target_tracker[2].track3( 0 , 1.0 / CtrlRateHz );
			target_angular_rate_Y = Target_tracker[2].x2;
			if( in_symmetry_range( target_angular_rate_Y , 0.001 ) && in_symmetry_range( angular_rate_ENU.z , 0.05 ) )
			{							
				Target_tracker[2].x1 = target_Yaw = Yaw;
				Yaw_ControlMode = Attitude_ControlMode_Angle;
			}
			
			break;
		}
	}
	
	//计算前馈量
		double YawAngleP =  ( Target_tracker[2].get_tracking_mode() == 4 ) ? ( Ps ) : 0;
		vector3<double> Tv1_ENU = { Ps*( Target_tracker_RP.x2.x - angular_rate_ENU.x ) + Target_tracker_RP.x3.x ,
															Ps*( Target_tracker_RP.x2.y - angular_rate_ENU.y ) + Target_tracker_RP.x3.y ,
															YawAngleP*( Target_tracker[2].x2 - angular_rate_ENU.z ) + Target_tracker[2].x3 };
		vector3<double> Tv2_ENU = { Ps*( Target_tracker_RP.x3.x - angular_acceleration_ENU.x ) + Target_tracker_RP.T4.x ,
															Ps*( Target_tracker_RP.x3.y - angular_acceleration_ENU.y ) + Target_tracker_RP.T4.y,
															YawAngleP*( Target_tracker[2].x3 - angular_acceleration_ENU.z ) + Target_tracker[2].x4 };
		
		vector3<double> Tv1;
		Tv1.x = Rotation_Matrix[0][0]*Tv1_ENU.x + Rotation_Matrix[0][1]*Tv1_ENU.y + Rotation_Matrix[0][2]*Tv1_ENU.z;
		Tv1.y = Rotation_Matrix[1][0]*Tv1_ENU.x + Rotation_Matrix[1][1]*Tv1_ENU.y + Rotation_Matrix[1][2]*Tv1_ENU.z;
		Tv1.z = Rotation_Matrix[2][0]*Tv1_ENU.x + Rotation_Matrix[2][1]*Tv1_ENU.y + Rotation_Matrix[2][2]*Tv1_ENU.z;
		vector3<double> Tv2;
		Tv2.x = Rotation_Matrix[0][0]*Tv2_ENU.x + Rotation_Matrix[0][1]*Tv2_ENU.y + Rotation_Matrix[0][2]*Tv2_ENU.z;
		Tv2.y = Rotation_Matrix[1][0]*Tv2_ENU.x + Rotation_Matrix[1][1]*Tv2_ENU.y + Rotation_Matrix[1][2]*Tv2_ENU.z;
		Tv2.z = Rotation_Matrix[2][0]*Tv2_ENU.x + Rotation_Matrix[2][1]*Tv2_ENU.y + Rotation_Matrix[2][2]*Tv2_ENU.z;
		vector3<double> Ta1 = { P2.x*( Tv1.x - angular_acceleration_ESO.x ) + Tv2.x ,
													P2.y*( Tv1.y - angular_acceleration_ESO.y ) + Tv2.y ,
													P2.z*( Tv1.z - angular_acceleration_ESO.z ) + Tv2.z };
	//计算前馈量
													
	//把目标速度从Bodyheading旋转到机体
		vector3<double> target_angular_rate_ENU;
		target_angular_rate_ENU.x = target_angular_rate_RP.x;
		target_angular_rate_ENU.y = target_angular_rate_RP.y;
		target_angular_rate_ENU.z = target_angular_rate_RP.z + target_angular_rate_Y;

		vector3<double> target_angular_rate_body;
		target_angular_rate_body.x = Rotation_Matrix[0][0]*target_angular_rate_ENU.x + Rotation_Matrix[0][1]*target_angular_rate_ENU.y + Rotation_Matrix[0][2]*target_angular_rate_ENU.z;
		target_angular_rate_body.y = Rotation_Matrix[1][0]*target_angular_rate_ENU.x + Rotation_Matrix[1][1]*target_angular_rate_ENU.y + Rotation_Matrix[1][2]*target_angular_rate_ENU.z;
		target_angular_rate_body.z = Rotation_Matrix[2][0]*target_angular_rate_ENU.x + Rotation_Matrix[2][1]*target_angular_rate_ENU.y + Rotation_Matrix[2][2]*target_angular_rate_ENU.z;
	//把目标速度从Bodyheading旋转到机体
													
	//计算目标角加速度
		vector3<double> target_angular_acceleration = target_angular_rate_body - angular_rate_ESO;
		target_angular_acceleration.x *= P2.x;
		target_angular_acceleration.y *= P2.y;
		target_angular_acceleration.z *= P2.z;
		target_angular_acceleration = target_angular_acceleration + Tv1;
	//计算目标角加速度
													
	//计算角加速度误差
	vector3<double> angular_acceleration_error = target_angular_acceleration - angular_acceleration_ESO;
	
	vector3<double> disturbance(
		ESO[0].get_EsDisturbance() ,
		ESO[1].get_EsDisturbance() ,
		ESO[2].get_EsDisturbance()
	);
	static vector3<double> last_disturbance = { 0 , 0 , 0 };		
	vector3<double> disturbance_Derivative = (disturbance - last_disturbance) * CtrlRateHz;
	last_disturbance = disturbance;
	
	double disturbance_x = disturbance_filter[0].run( disturbance_Derivative.x );
	double disturbance_y = disturbance_filter[1].run( disturbance_Derivative.y );
	double disturbance_z = disturbance_filter[2].run( disturbance_Derivative.z );
	
	double outRoll;double outPitch;double outYaw;
	if( inFlight )
	{
		outRoll = 	( ESO[0].get_EsMainPower() + ESO[0].T * ( angular_acceleration_error.x * P3.x + Ta1.x /*- disturbance_x*/ ) )/ESO[0].b;
		outPitch =	( ESO[1].get_EsMainPower() + ESO[1].T * ( angular_acceleration_error.y * P3.y + Ta1.y /*- disturbance_y*/ ) )/ESO[1].b;
//		outYaw =		( ESO_AngularRate_get_EsMainPower( &ESO[2] ) + ESO[2].T * ( angular_acceleration_error.z * P.z + Ta1.z /*- disturbance_z*/ ) )/ESO[2].b;
		outYaw = ( target_angular_acceleration.z - disturbance.z ) / ESO[2].b;
	}
	else
	{
		outRoll = 	ESO[0].T * ( angular_acceleration_error.x * P3.x )/ESO[0].b;
		outPitch =	ESO[1].T * ( angular_acceleration_error.y * P3.y )/ESO[1].b;
		//outYaw =		ESO[2].T * ( angular_acceleration_error.z * P.z )/ESO[2].b;
		outYaw = ( target_angular_acceleration.z ) / ESO[2].b;
	}
	
	switch( uav_type )
	{
		case UAVType_Rotor4_X:
			ctrl_Attitude_MultiRotor_X4_PWM( outRoll , outPitch , outYaw );
			break;		
		
		case UAVType_Rotor6_X:
			ctrl_Attitude_MultiRotor_X6_PWM( outRoll , outPitch , outYaw );
			break;
		
		case UAVType_Rotor8_X:
			ctrl_Attitude_MultiRotor_X8_PWM( outRoll , outPitch , outYaw );
			break;
//		
//		case UAVType_Rotor4_C:
//			ctrl_Attitude_MultiRotor_C4_PWM( outRoll , outPitch , outYaw );
//			break;
//		
//		case UAVType_Rotor42_C:
//			ctrl_Attitude_MultiRotor_C42_PWM( outRoll , outPitch , outYaw );
//			break;
		case UAVType_Rotor6_S1:
			ctrl_Attitude_MultiRotor_S6_1_PWM( outRoll , outPitch , outYaw );
			break;
		
		default:
			PWM_PullDownAll();
			break;
	}
}
	
//电机非线性输出 线性修正
static inline void throttle_nonlinear_compensation( double out[8] )
{
	double output_minimum_throttle = STThrottle;
	double output_range = 100.0f - output_minimum_throttle;
	double inv_output_range = 1.0 / output_range;
	
	//a：非线性因子(0-1)
	//m：最大油门比例(0.6-1)
	
	//设油门-力曲线方程为：
	//F = kx^2 + (1-a)x ( 0<=x<=m F最大值为1 )
	//x = m时：km^2 + (1-a)m = 1
	//得k = ( 1 - (1-a)m ) / m^2
	//a_1 = a - 1
	//Hk  = 1 / 2k
	//K4  = 4* k
	//解方程组：kx^2 + (1-a)x = out
	//得到的x即为线性化后的输出
	double _lift_max = FullThrRatio;
	double a_1 = NonlinearFactor - 1;
	double k = ( 1 + a_1*_lift_max ) / (_lift_max*_lift_max);
	double Hk = 1.0f / (2*k);
	double K4 = 4 * k;
		
	for( uint8_t i = 0; i < 8; ++i )
	{
		if( out[i] > output_minimum_throttle - 0.1f )
		{
			out[i] -= output_minimum_throttle;
			out[i] *= inv_output_range;
			if( out[i] < 0 )
				out[i] = 0;
			out[i] = Hk*( a_1 + safe_sqrt( a_1*a_1 + K4*out[i] ) );
			out[i] *= output_range;
			out[i] += output_minimum_throttle;			
		}
		else
			out[i] = 0;
	}
}

//四旋翼X模式
static void ctrl_Attitude_MultiRotor_X4_PWM( double outRoll , double outPitch , double outYaw )
{
	double rotor_output[8] = {0,0,0,0,0,0,0,0};

	double output_minimum_throttle = STThrottle;
//	if( get_current_Receiver()->data[0] < 5 )
//	{
//		PWM_PullDownAll();
//		update_output_throttle( 0 , 1.0f/CtrlRateHz );
//		return;
//	}		
	
	if( throttle < output_minimum_throttle - 0.1f )
	{
		PWM_PullDownAll();
		update_output_throttle( 0 , 1.0/CtrlRateHz );
		return;
	}		
			
	double output_throttle = throttle;
	double output_midpoint = ( 100.0f - output_minimum_throttle ) / 2;
//		double cos_angle = lean_cos;
//		if( cos_angle < 0.5f )cos_angle = 0.5f;
//		output_throttle = throttle / cos_angle;
	
	/*pitch roll 输出限幅*/
		//如果需要的pitch roll输出超出当前油门能提供的输出范围
		//调整油门获得尽量满足pitch roll输出
		rotor_output[0] = -outPitch+outRoll;
		rotor_output[1] = +outPitch+outRoll;		
		rotor_output[2] = +outPitch-outRoll;
		rotor_output[3] = -outPitch-outRoll;
		double output_max;
		double output_min ;
		output_max = rotor_output[0];
		for( int i = 1 ; i < 4 ; ++i )
		{
			if( rotor_output[i] > output_max ) output_max = rotor_output[i];
		}
		
		double max_allow_output = 100.0f - output_throttle;
		double min_allow_output = output_minimum_throttle - output_throttle;			
		double allow_ouput_range;
		if( max_allow_output < -min_allow_output )
		{
			//降低油门确保姿态输出
			allow_ouput_range = max_allow_output;
			if( output_max > allow_ouput_range )
			{
				if( output_max > output_midpoint )
				{
					output_throttle = output_midpoint + output_minimum_throttle;
					allow_ouput_range = output_midpoint;
				}
				else
				{
					output_throttle = 100.0f - output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		else
		{
			//抬高油门保证姿态输出
			allow_ouput_range = -min_allow_output;
			if( output_max > allow_ouput_range )
			{
				double hover_throttle_force = hover_throttle - output_minimum_throttle;
				double max_allowed_output_range = ( hover_throttle_force > output_midpoint ? output_midpoint : hover_throttle_force ) * 0.95;
				
//				double hover_throttle_force = hover_throttle - output_minimum_throttle;
//				double max_allowed_throttle_force = output_throttle - output_minimum_throttle;
//				max_allowed_throttle_force += hover_throttle_force*0.3;
//				double max_allowed_output_range = ( max_allowed_throttle_force > output_midpoint ? output_midpoint : max_allowed_throttle_force );
				if( max_allowed_output_range < output_throttle - output_minimum_throttle )
					max_allowed_output_range = output_throttle - output_minimum_throttle;
				double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
				if( output_max > max_allowed_output_range )
				{
					output_throttle = max_allowed_throttle;
					allow_ouput_range = max_allowed_output_range;
				}
				else
				{
					output_throttle = output_minimum_throttle + output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		
		//输出限幅修正
		if( output_max > allow_ouput_range )
		{
			double scale  = allow_ouput_range / output_max;
			rotor_output[0] *= scale;
			rotor_output[1] *= scale;
			rotor_output[2] *= scale;
			rotor_output[3] *= scale;
			ESO[0].update_u( outRoll * scale );
			ESO[1].update_u( outPitch * scale );
		}		
		else
		{
			ESO[0].update_u( outRoll );
			ESO[1].update_u( outPitch );
		}
	/*pitch roll 输出限幅*/
	
	/*yaw output 输出限幅*/
		//for Yaw control, it has the lowest priority
		//lower output to ensure attitude control and alt control 
		output_max = 0.0f;
		output_min = 100.0f;

		/*find min yaw_scale*/
			double yaw_scale = 1.0f;
		
			for( int i = 0 ; i < 4 ; ++i )
			{
				double current_out_yaw = ( (i&1) == 1 ) ? outYaw : -outYaw;
				
				double current_rotor_output = output_throttle + rotor_output[i];
				max_allow_output = 100.0f - current_rotor_output;
				min_allow_output = output_minimum_throttle - current_rotor_output;
				if( current_out_yaw > max_allow_output + 0.01f )
				{
					double new_yaw_scale = max_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
				else if( current_out_yaw < min_allow_output - 0.01f )
				{
					double new_yaw_scale = min_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
			}
						
		/*find min yaw_scale*/
		
		//lower yaw output to ensure attitude control and alt control
		if( yaw_scale < 0 )
			yaw_scale = 0;
		outYaw *= yaw_scale;
		ESO[2].update_u( outYaw );
	/*yaw output 输出限幅*/
		
	update_output_throttle( output_throttle , 1.0/CtrlRateHz );
	rotor_output[0] += output_throttle-outYaw;
	rotor_output[1] += output_throttle+outYaw;
	rotor_output[2] += output_throttle-outYaw;
	rotor_output[3] += output_throttle+outYaw;
	
	throttle_nonlinear_compensation( rotor_output );
	PWM_Out( rotor_output );
}

//六旋翼X模式
static void ctrl_Attitude_MultiRotor_X6_PWM( double outRoll , double outPitch , double outYaw )
{
	double rotor_output[8] = {0,0,0,0,0,0,0,0};

	double output_minimum_throttle = STThrottle;
//	if( get_current_Receiver()->data[0] < 5 )
//	{
//		PWM_PullDownAll();
//		update_output_throttle( 0 , 1.0f/CtrlRateHz );
//		return;
//	}		
	
	if( throttle < output_minimum_throttle - 0.1f )
	{
		PWM_PullDownAll();
		update_output_throttle( 0 , 1.0/CtrlRateHz );
		return;
	}
			
	double output_throttle = throttle;
	double output_midpoint = ( 100.0f - output_minimum_throttle ) / 2;
//		float cos_angle = lean_cos;
//		if( cos_angle < 0.5f )cos_angle = 0.5f;
//		output_throttle = throttle / cos_angle;
	
	/*pitch roll 输出限幅*/
		//如果需要的pitch roll输出超出当前油门能提供的输出范围
		//调整油门获得尽量满足pitch roll输出
		double RollS = outRoll * 1.1547005383792515290182975610039f;
		double half_outRoll = 0.5f * RollS;
		rotor_output[0] = -outPitch+half_outRoll;
		rotor_output[1] = RollS;
		rotor_output[2] = +outPitch+half_outRoll;
		rotor_output[3] = +outPitch-half_outRoll;
		rotor_output[4] = -RollS;
		rotor_output[5] = -outPitch-half_outRoll;
	
		double output_max;
		double output_min ;
		output_max = rotor_output[0];
		for( int i = 1 ; i < 6 ; ++i )
		{
			if( rotor_output[i] > output_max ) output_max = rotor_output[i];
		}
		
		double max_allow_output = 100.0f - output_throttle;
		double min_allow_output = output_minimum_throttle - output_throttle;			
		double allow_ouput_range;
		if( max_allow_output < -min_allow_output )
		{
			//降低油门确保姿态输出
			allow_ouput_range = max_allow_output;
			if( output_max > allow_ouput_range )
			{
				if( output_max > output_midpoint )
				{
					output_throttle = output_midpoint + output_minimum_throttle;
					allow_ouput_range = output_midpoint;
				}
				else
				{
					output_throttle = 100.0f - output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		else
		{
			allow_ouput_range = -min_allow_output;
			if( output_max > allow_ouput_range )
			{
				double hover_throttle_force = hover_throttle - output_minimum_throttle;
				double max_allowed_output_range = ( hover_throttle_force > output_midpoint ? output_midpoint : hover_throttle_force ) * 0.8f;
				if( max_allowed_output_range < output_throttle - output_minimum_throttle )
					max_allowed_output_range = output_throttle - output_minimum_throttle;
				double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
				if( output_max > max_allowed_output_range )
				{
					output_throttle = max_allowed_throttle;
					allow_ouput_range = max_allowed_output_range;
				}
				else
				{
					output_throttle = output_minimum_throttle + output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		
		if( output_max > allow_ouput_range )
		{
			double scale  = allow_ouput_range / output_max;
			rotor_output[0] *= scale;
			rotor_output[1] *= scale;
			rotor_output[2] *= scale;
			rotor_output[3] *= scale;
			rotor_output[4] *= scale;
			rotor_output[5] *= scale;
			ESO[0].update_u( outRoll * scale );
			ESO[1].update_u( outPitch * scale );
		}		
		else
		{
			ESO[0].update_u( outRoll );
			ESO[1].update_u( outPitch );
		}
	/*pitch roll 输出限幅*/
	
	/*yaw output 输出限幅*/
		//for Yaw control, it has the lowest priority
		//lower output to ensure attitude control and alt control 
		output_max = 0.0f;
		output_min = 100.0f;

		/*find min yaw_scale*/
			double yaw_scale = 1.0f;
		
			for( int i = 0 ; i < 6 ; ++i )
			{
				double current_out_yaw = ( (i&1) == 1 ) ? outYaw : -outYaw;
				
				double current_rotor_output = output_throttle + rotor_output[i];
				max_allow_output = 100.0f - current_rotor_output;
				min_allow_output = output_minimum_throttle - current_rotor_output;
				if( current_out_yaw > max_allow_output + 0.01f )
				{
					double new_yaw_scale = max_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
				else if( current_out_yaw < min_allow_output - 0.01f )
				{
					double new_yaw_scale = min_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
			}
						
		/*find min yaw_scale*/
		
		//lower yaw output to ensure attitude control and alt control
		if( yaw_scale < 0 )
			yaw_scale = 0;
		outYaw *= yaw_scale;
		ESO[2].update_u( outYaw );
	/*yaw output 输出限幅*/
		
	update_output_throttle( output_throttle , 1.0/CtrlRateHz );
	rotor_output[0] += output_throttle-outYaw;
	rotor_output[1] += output_throttle+outYaw;
	rotor_output[2] += output_throttle-outYaw;
	rotor_output[3] += output_throttle+outYaw;
	rotor_output[4] += output_throttle-outYaw;
	rotor_output[5] += output_throttle+outYaw;
	
	throttle_nonlinear_compensation( rotor_output );
	PWM_Out( rotor_output );
}

//六旋翼异构
/*
	o--o
	o--o
	o--o
*/
static void ctrl_Attitude_MultiRotor_S6_1_PWM( double outRoll , double outPitch , double outYaw )
{
	double rotor_output[8] = {0,0,0,0,0,0,0,0};

	double output_minimum_throttle = STThrottle;
//	if( get_current_Receiver()->data[0] < 5 )
//	{
//		PWM_PullDownAll();
//		update_output_throttle( 0 , 1.0f/CtrlRateHz );
//		return;
//	}		
	
	if( throttle < output_minimum_throttle - 0.1f )
	{
		PWM_PullDownAll();
		update_output_throttle( 0 , 1.0/CtrlRateHz );
		return;
	}
			
	double output_throttle = throttle;
	double output_midpoint = ( 100.0f - output_minimum_throttle ) / 2;
//		float cos_angle = lean_cos;
//		if( cos_angle < 0.5f )cos_angle = 0.5f;
//		output_throttle = throttle / cos_angle;
	
	/*pitch roll 输出限幅*/
		//如果需要的pitch roll输出超出当前油门能提供的输出范围
		//调整油门获得尽量满足pitch roll输出
		double RollS = outRoll * 1.732;
		double half_outRoll = 0.5f * RollS;
		rotor_output[0] = -outPitch+half_outRoll;
		rotor_output[1] = RollS;
		rotor_output[2] = +outPitch+half_outRoll;
		rotor_output[3] = +outPitch-half_outRoll;
		rotor_output[4] = -RollS;
		rotor_output[5] = -outPitch-half_outRoll;
	
		double output_max;
		double output_min ;
		output_max = rotor_output[0];
		for( int i = 1 ; i < 6 ; ++i )
		{
			if( rotor_output[i] > output_max ) output_max = rotor_output[i];
		}
		
		double max_allow_output = 100.0f - output_throttle;
		double min_allow_output = output_minimum_throttle - output_throttle;			
		double allow_ouput_range;
		if( max_allow_output < -min_allow_output )
		{
			//降低油门确保姿态输出
			allow_ouput_range = max_allow_output;
			if( output_max > allow_ouput_range )
			{
				if( output_max > output_midpoint )
				{
					output_throttle = output_midpoint + output_minimum_throttle;
					allow_ouput_range = output_midpoint;
				}
				else
				{
					output_throttle = 100.0f - output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		else
		{
			allow_ouput_range = -min_allow_output;
			if( output_max > allow_ouput_range )
			{
				double hover_throttle_force = hover_throttle - output_minimum_throttle;
				double max_allowed_output_range = ( hover_throttle_force > output_midpoint ? output_midpoint : hover_throttle_force ) * 0.8f;
				if( max_allowed_output_range < output_throttle - output_minimum_throttle )
					max_allowed_output_range = output_throttle - output_minimum_throttle;
				double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
				if( output_max > max_allowed_output_range )
				{
					output_throttle = max_allowed_throttle;
					allow_ouput_range = max_allowed_output_range;
				}
				else
				{
					output_throttle = output_minimum_throttle + output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		
		if( output_max > allow_ouput_range )
		{
			double scale  = allow_ouput_range / output_max;
			rotor_output[0] *= scale;
			rotor_output[1] *= scale;
			rotor_output[2] *= scale;
			rotor_output[3] *= scale;
			rotor_output[4] *= scale;
			rotor_output[5] *= scale;
			ESO[0].update_u( outRoll * scale );
			ESO[1].update_u( outPitch * scale );
		}		
		else
		{
			ESO[0].update_u( outRoll );
			ESO[1].update_u( outPitch );
		}
	/*pitch roll 输出限幅*/
	
	/*yaw output 输出限幅*/
		//for Yaw control, it has the lowest priority
		//lower output to ensure attitude control and alt control 
		output_max = 0.0f;
		output_min = 100.0f;

		/*find min yaw_scale*/
			double yaw_scale = 1.0f;
		
			for( int i = 0 ; i < 6 ; ++i )
			{
				double current_out_yaw = ( (i&1) == 1 ) ? 0.5*outYaw : -0.5*outYaw;
				if( i==1 || i==4 )
					current_out_yaw *= 2;
				
				double current_rotor_output = output_throttle + rotor_output[i];
				max_allow_output = 100.0f - current_rotor_output;
				min_allow_output = output_minimum_throttle - current_rotor_output;
				if( current_out_yaw > max_allow_output + 0.01f )
				{
					double new_yaw_scale = max_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
				else if( current_out_yaw < min_allow_output - 0.01f )
				{
					double new_yaw_scale = min_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
			}
						
		/*find min yaw_scale*/
		
		//lower yaw output to ensure attitude control and alt control
		if( yaw_scale < 0 )
			yaw_scale = 0;
		outYaw *= yaw_scale;
		ESO[2].update_u( outYaw );
	/*yaw output 输出限幅*/
		
	update_output_throttle( output_throttle , 1.0/CtrlRateHz );
	rotor_output[0] += output_throttle-outYaw;
	rotor_output[1] += output_throttle+outYaw;
	rotor_output[2] += output_throttle-outYaw;
	rotor_output[3] += output_throttle+outYaw;
	rotor_output[4] += output_throttle-outYaw;
	rotor_output[5] += output_throttle+outYaw;
	
	throttle_nonlinear_compensation( rotor_output );
	PWM_Out( rotor_output );
}

//八旋翼X模式
static void ctrl_Attitude_MultiRotor_X8_PWM( double outRoll , double outPitch , double outYaw )
{
	double rotor_output[8] = {0,0,0,0,0,0,0,0};

	double output_minimum_throttle = STThrottle;
//	if( get_current_Receiver()->data[0] < 5 )
//	{
//		PWM_PullDownAll();
//		update_output_throttle( 0 , 1.0f/CtrlRateHz );
//		return;
//	}		
	
	if( throttle < output_minimum_throttle - 0.1f )
	{
		PWM_PullDownAll();
		update_output_throttle( 0 , 1.0/CtrlRateHz );
		return;
	}
			
	double output_throttle = throttle;
	double output_midpoint = ( 100.0f - output_minimum_throttle ) / 2;
//		float cos_angle = lean_cos;
//		if( cos_angle < 0.5f )cos_angle = 0.5f;
//		output_throttle = throttle / cos_angle;
	
	/*pitch roll 输出限幅*/
		//如果需要的pitch roll输出超出当前油门能提供的输出范围
		//调整油门获得尽量满足pitch roll输出
		rotor_output[0] = -outPitch+outRoll;
		rotor_output[1] = -outPitch+outRoll;
		rotor_output[2] = +outPitch+outRoll;
		rotor_output[3] = +outPitch+outRoll;
		rotor_output[4] = +outPitch-outRoll;
		rotor_output[5] = +outPitch-outRoll;
		rotor_output[6] = -outPitch-outRoll;
		rotor_output[7] = -outPitch-outRoll;
	
		double output_max;
		double output_min ;
		output_max = rotor_output[0];
		for( int i = 1 ; i < 8 ; ++i )
		{
			if( rotor_output[i] > output_max ) output_max = rotor_output[i];
		}
		
		double max_allow_output = 100.0f - output_throttle;
		double min_allow_output = output_minimum_throttle - output_throttle;			
		double allow_ouput_range;
		if( max_allow_output < -min_allow_output )
		{
			//降低油门确保姿态输出
			allow_ouput_range = max_allow_output;
			if( output_max > allow_ouput_range )
			{
				if( output_max > output_midpoint )
				{
					output_throttle = output_midpoint + output_minimum_throttle;
					allow_ouput_range = output_midpoint;
				}
				else
				{
					output_throttle = 100.0f - output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		else
		{
			allow_ouput_range = -min_allow_output;
			if( output_max > allow_ouput_range )
			{
				double hover_throttle_force = hover_throttle - output_minimum_throttle;
				double max_allowed_output_range = ( hover_throttle_force > output_midpoint ? output_midpoint : hover_throttle_force ) * 0.8f;
				if( max_allowed_output_range < output_throttle - output_minimum_throttle )
					max_allowed_output_range = output_throttle - output_minimum_throttle;
				double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
				if( output_max > max_allowed_output_range )
				{
					output_throttle = max_allowed_throttle;
					allow_ouput_range = max_allowed_output_range;
				}
				else
				{
					output_throttle = output_minimum_throttle + output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		
		if( output_max > allow_ouput_range )
		{
			double scale  = allow_ouput_range / output_max;
			rotor_output[0] *= scale;
			rotor_output[1] *= scale;
			rotor_output[2] *= scale;
			rotor_output[3] *= scale;
			rotor_output[4] *= scale;
			rotor_output[5] *= scale;
			rotor_output[6] *= scale;
			rotor_output[7] *= scale;
			ESO[0].update_u( outRoll * scale );
			ESO[1].update_u( outPitch * scale );
		}		
		else
		{
			ESO[0].update_u( outRoll );
			ESO[1].update_u( outPitch );
		}
	/*pitch roll 输出限幅*/
	
	/*yaw output 输出限幅*/
		//for Yaw control, it has the lowest priority
		//lower output to ensure attitude control and alt control 
		output_max = 0.0f;
		output_min = 100.0f;

		/*find min yaw_scale*/
			double yaw_scale = 1.0f;
		
			for( int i = 0 ; i < 8 ; ++i )
			{
				double current_out_yaw = ( (i&1) == 1 ) ? outYaw : -outYaw;
				
				double current_rotor_output = output_throttle + rotor_output[i];
				max_allow_output = 100.0f - current_rotor_output;
				min_allow_output = output_minimum_throttle - current_rotor_output;
				if( current_out_yaw > max_allow_output + 0.01f )
				{
					double new_yaw_scale = max_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
				else if( current_out_yaw < min_allow_output - 0.01f )
				{
					double new_yaw_scale = min_allow_output / current_out_yaw;
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
			}
						
		/*find min yaw_scale*/
		
		//lower yaw output to ensure attitude control and alt control
		if( yaw_scale < 0 )
			yaw_scale = 0;
		outYaw *= yaw_scale;
		ESO[2].update_u( outYaw );
	/*yaw output 输出限幅*/
		
	update_output_throttle( output_throttle , 1.0/CtrlRateHz );
	rotor_output[0] += output_throttle-outYaw;
	rotor_output[1] += output_throttle+outYaw;
	rotor_output[2] += output_throttle-outYaw;
	rotor_output[3] += output_throttle+outYaw;
	rotor_output[4] += output_throttle-outYaw;
	rotor_output[5] += output_throttle+outYaw;
	rotor_output[6] += output_throttle-outYaw;
	rotor_output[7] += output_throttle+outYaw;
	
	throttle_nonlinear_compensation( rotor_output );
	PWM_Out( rotor_output );
}

void init_Ctrl_Attitude()
{
	//注册参数
	AttCtrlCfg initial_cfg;
	initial_cfg.UAVType = UAVType_Rotor4_X;
	initial_cfg.STThrottle[0] = 10;
	initial_cfg.NonlinearFactor[0] = 0.65;
	initial_cfg.FullThrRatio[0] = 0.95;
	initial_cfg.T[0] = 0.1;
	initial_cfg.b[0] = 5.5;	initial_cfg.b[2] = 5.5;	initial_cfg.b[4] = 1.0;
	initial_cfg.TD4_P1[0] = 25;	initial_cfg.TD4_P1[2] = 25;	initial_cfg.TD4_P1[4] = 2;
	initial_cfg.TD4_P2[0] = 25;	initial_cfg.TD4_P2[2] = 25;	initial_cfg.TD4_P2[4] = 5;
	initial_cfg.TD4_P3[0] = 25;	initial_cfg.TD4_P3[2] = 25;	initial_cfg.TD4_P3[4] = 25;
	initial_cfg.TD4_P4[0] = 25;	initial_cfg.TD4_P4[2] = 25;	initial_cfg.TD4_P4[4] = 25;
	initial_cfg.P1[0] = 15;	initial_cfg.P1[2] = 15;	initial_cfg.P1[4] = 2;
	initial_cfg.P2[0] = 15;	initial_cfg.P2[2] = 15;	initial_cfg.P2[4] = 5;
	initial_cfg.P3[0] = 15;	initial_cfg.P3[2] = 15;	initial_cfg.P3[4] = 15;
	initial_cfg.P4[0] = 15;	initial_cfg.P4[2] = 15;	initial_cfg.P4[4] = 15;
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT8 ,	//UAV Type
		MAV_PARAM_TYPE_REAL32 ,	//起转油门
		MAV_PARAM_TYPE_REAL32 ,	//非线性参数
		MAV_PARAM_TYPE_REAL32 ,	//满油门比例
		MAV_PARAM_TYPE_REAL32 ,	//T
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//b[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//TD4_P1[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//TD4_P2[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//TD4_P3[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//TD4_P4[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//P1[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//P2[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//P3[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//P4[3]
	};
	SName param_names[] = {
		"AC_UAVType" ,	//UAV Type
		"AC_STThr" ,	//起转油门
		"AC_NonlinF" ,	//非线性参数
		"AC_FullThrR" ,	//满油门比例
		"AC_T" ,	//T
		"AC_Roll_b" ,"AC_Pitch_b" ,"AC_Yaw_b" ,	//b[3]
		"AC_Roll_TD4P1" ,"AC_Pitch_TD4P1" ,"AC_Yaw_TD4P1" ,	//TD4_P1[3]
		"AC_Roll_TD4P2" ,"AC_Pitch_TD4P2" ,"AC_Yaw_TD4P2" ,	//TD4_P2[3]
		"AC_Roll_TD4P3" ,"AC_Pitch_TD4P3" ,"AC_Yaw_TD4P3" ,	//TD4_P3[3]
		"AC_Roll_TD4P4" ,"AC_Pitch_TD4P4" ,"AC_Yaw_TD4P4" ,	//TD4_P4[3]
		"AC_Roll_P1" ,"AC_Pitch_P1" ,"AC_Yaw_P1" ,	//P1[3]
		"AC_Roll_P2" ,"AC_Pitch_P2" ,"AC_Yaw_P2" ,	//P2[3]
		"AC_Roll_P3" ,"AC_Pitch_P3" ,"AC_Yaw_P3" ,	//P3[3]
		"AC_Roll_P4" ,"AC_Pitch_P4" ,"AC_Yaw_P4" ,	//P4[3]
	};
	ParamGroupRegister( "AttCtrl", 1, 32, param_types, param_names, (uint64_t*)&initial_cfg );
	
	//初始化滤波器
	disturbance_filter[0].set_cutoff_frequency( CtrlRateHz, 10 );
	disturbance_filter[1].set_cutoff_frequency( CtrlRateHz, 10 );
	disturbance_filter[2].set_cutoff_frequency( CtrlRateHz, 10 );
}