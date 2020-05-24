#include "ctrl_Attitude.hpp"
#include "ControlSystem.hpp"
#include "ctrl_Main.hpp"
#include "Parameters.hpp"
#include "MeasurementSystem.hpp"
#include "Sensors.hpp"
#include "TD4.hpp"
#include "ESO_AngularRate.hpp"
#include "ESO_h.hpp"
#include "Filters_LP.hpp"
#include "drv_PWMOut.hpp"
#include "smooth_kp.hpp"
#include "TD3_3D.hpp"
#include "vector2.hpp"

/*参数*/
	//控制参数
	struct PosCtrlCfg
	{
		//XY航线速度
		float AutoVXY[2];
		//Z航线速度
		float AutoVZ[2];
		//XYZ航线速度
		float AutoVXYZ[2];
		
		//高度前馈滤波器
		float Z_TD4P1[2];
		float Z_TD4P2[2];
		float Z_TD4P3[2];
		float Z_TD4P4[2];
	}__PACKED;
/*参数*/	

/*控制接口*/
	static bool Altitude_Control_Enabled = false;
	static bool Position_Control_Enabled = false;
	
	//位置控制模式
	static Position_ControlMode Altitude_ControlMode = Position_ControlMode_Position;
	static Position_ControlMode HorizontalPosition_ControlMode = Position_ControlMode_Position;
	
	//期望TD4滤波器
	static TD4 Target_tracker[3];
	//期望速度低通滤波器
	static Filter_Butter4_LP TargetVelocityFilter[3];
	//悬停油门低频滤波器
	static TD4_Lite HoverThrottleFilter;
	
	static vector3<double> target_position;
	static vector3<double> target_velocity;
	static double VelCtrlMaxRoll = -1 , VelCtrlMaxPitch = -1;
	
	/*高度*/
		bool is_Altitude_Control_Enabled( bool* ena, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				*ena = Altitude_Control_Enabled;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Altitude_Control_Enable( double TIMEOUT )
		{
			if( get_Altitude_MSStatus() != MS_Ready )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == true )
				{	//控制器已打开
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				Attitude_Control_Enable();
				Altitude_ControlMode = Position_ControlMode_Locking;
				Altitude_Control_Enabled = true;
				
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				//读参数
				PosCtrlCfg cfg;
				if( ReadParamGroup( "PosCtrl", (uint64_t*)&cfg, 0, TIMEOUT ) != PR_OK )
				{
					UnlockCtrl();
					return false;
				}				
				Position_Control_set_ZAutoSpeed(cfg.AutoVZ[0]);
				Target_tracker[2].P1 = cfg.Z_TD4P1[0];
				Target_tracker[2].P2 = cfg.Z_TD4P2[0];
				Target_tracker[2].P3 = cfg.Z_TD4P3[0];
				Target_tracker[2].P4 = cfg.Z_TD4P4[0];
				
				HoverThrottleFilter.reset();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Altitude_Control_Disable( double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( ForceMSafeCtrl && !isMSafe )
				{	//屏蔽用户控制
					UnlockCtrl();
					return false;
				}
				Position_Control_Disable();
				Altitude_ControlMode = Position_ControlMode_Null;
				Altitude_Control_Enabled = false;					
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		bool get_Altitude_ControlMode( Position_ControlMode* mode, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && (Is_2DAutoMode(Altitude_ControlMode) || Is_3DAutoMode(Altitude_ControlMode)) )
				{	//用户控制访问且为自动模式时更新控制时间
					last_ZCtrlTime = TIME::now();
				}
				*mode = Altitude_ControlMode;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//设定Z自动飞行速度
		static double AutoVelZ = 200;
		bool Position_Control_set_ZAutoSpeed( double AtVelZ, double TIMEOUT )
		{
			if( isnan(AtVelZ) || isinf(AtVelZ) )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				AutoVelZ = AtVelZ;
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//移动Z目标位置
		bool Position_Control_move_TargetPositionZRelative( double posz, double TIMEOUT )
		{
			if( isnan(posz) || isinf(posz) )
				return false;
			//必须为位置或者自动模式
			if( Altitude_ControlMode!=Position_ControlMode_Position &&
					Is_2DAutoMode(Altitude_ControlMode)==false &&
					Is_3DAutoMode(Altitude_ControlMode)==false )
				return false;
			
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if( !isMSafe )
			{	//用户控制访问更新控制时间
				last_ZCtrlTime = TIME::now();
			}
			target_position.z += posz;
			
			return true;
		}
		
		bool Position_Control_set_TargetPositionZ( double posz, double TIMEOUT )
		{
			if( isnan(posz) || isinf(posz) )
				return false;
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				target_position.z = posz;
				if( Is_2DAutoMode(Altitude_ControlMode)==false && Altitude_ControlMode!=Position_ControlMode_Position )
				{
					vector3<double> pos;
					get_Position(&pos);
					Target_tracker[2].x1 = pos.z;
				}
				Altitude_ControlMode = Position_ControlMode_RouteLine;
			
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_TargetPositionZGlobal( double posz, double TIMEOUT )
		{
			if( isnan(posz) || isinf(posz) )
				return false;
			
			//获取最优全球定位传感器信息
			PosSensorHealthInf1 global_inf;
			if( get_OptimalGlobal_Z( &global_inf ) == false )
				return false;
			posz -= global_inf.HOffset;
			return Position_Control_set_TargetPositionZ( posz, TIMEOUT );
		}
		bool Position_Control_set_TargetPositionZRelative( double posz, double TIMEOUT )
		{
			if( isnan(posz) || isinf(posz) )
				return false;
			vector3<double> pos;
			if( get_Position( &pos, TIMEOUT ) == false )
				return false;
			return Position_Control_set_TargetPositionZ( pos.z + posz, TIMEOUT );
		}
		bool Position_Control_set_TargetVelocityZ( double velz, double TIMEOUT )
		{
			if( isnan(velz) || isinf(velz) )
				return false;
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				target_velocity.z = velz;
				Altitude_ControlMode = Position_ControlMode_Velocity;
			
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_TargetPositionZRA( double posz, double TIMEOUT )
		{
			//获取起飞位置Z坐标
			double homeZ;
			getHomeLocalZ(&homeZ);
			return Position_Control_set_TargetPositionZ( homeZ + posz, TIMEOUT );
		}
		bool Position_Control_set_ZLock( double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				if( Altitude_ControlMode != Position_ControlMode_Position )
					Altitude_ControlMode = Position_ControlMode_Locking;
			
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//起飞到当前高度上方的height高度
		static double TakeoffHeight;
		bool Position_Control_Takeoff_HeightRelative( double height, double TIMEOUT )
		{
			if( height < 10 )
					return false;
			bool inFlight;	get_is_inFlight(&inFlight);
			if( inFlight == true )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}	
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				
				Altitude_ControlMode = Position_ControlMode_Takeoff;
				TakeoffHeight = height;
			
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_Takeoff_HeightGlobal( double height, double TIMEOUT )
		{
			//获取最优全球定位传感器信息
			PosSensorHealthInf1 global_inf;
			if( get_OptimalGlobal_Z( &global_inf ) == false )
				return false;
			height -= global_inf.HOffset;
			height -= global_inf.PositionENU.z;
			return Position_Control_Takeoff_HeightRelative( height, TIMEOUT );
		}
		bool Position_Control_Takeoff_Height( double height, double TIMEOUT )
		{
			vector3<double> pos;
			get_Position(&pos);
			height = height - pos.z;
			return Position_Control_Takeoff_HeightRelative( height, TIMEOUT );
		}
	/*高度*/
		
	/*水平位置*/
		bool is_Position_Control_Enabled( bool* ena, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				*ena = Position_Control_Enabled;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_Enable( double TIMEOUT )
		{
			if( get_Position_MSStatus() != MS_Ready )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == true )
				{	//控制器已打开
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				Altitude_Control_Enable();
				HorizontalPosition_ControlMode = Position_ControlMode_Locking;
				Position_Control_Enabled = true;
				
				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
				
				//读参数
				PosCtrlCfg cfg;
				if( ReadParamGroup( "PosCtrl", (uint64_t*)&cfg, 0, TIMEOUT ) != PR_OK )
				{
					UnlockCtrl();
					return false;
				}				
				Position_Control_set_XYAutoSpeed(cfg.AutoVXY[0]);
				Position_Control_set_XYZAutoSpeed(cfg.AutoVXYZ[0]);
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_Disable( double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( ForceMSafeCtrl && !isMSafe )
				{	//屏蔽用户控制
					UnlockCtrl();
					return false;
				}
				HorizontalPosition_ControlMode = Position_ControlMode_Null;
				Position_Control_Enabled = false;		
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		bool get_Position_ControlMode( Position_ControlMode* mode, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && (Is_2DAutoMode(HorizontalPosition_ControlMode) || Is_3DAutoMode(HorizontalPosition_ControlMode)) )
				{	//用户控制访问且为自动模式时更新控制时间
					last_XYCtrlTime = TIME::now();
				}
				*mode = HorizontalPosition_ControlMode;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//设定自动飞行速度
		static double AutoVelXY = 500;
		static double AutoVelXYZ = 500;
		bool Position_Control_set_XYAutoSpeed( double AtVelXY, double TIMEOUT )
		{
			if( isnan(AtVelXY) || isinf(AtVelXY) )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				AutoVelXY = AtVelXY;
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_XYZAutoSpeed( double AtVelXYZ, double TIMEOUT )
		{
			if( isnan(AtVelXYZ) || isinf(AtVelXYZ) )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				AutoVelXYZ = AtVelXYZ;
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		bool Position_Control_set_TargetVelocityXY_AngleLimit( double velx, double vely, double maxAngle, double TIMEOUT )
		{
			if( isnan(velx) || isinf(velx) ||
					isnan(vely) || isinf(vely) ||
					isnan(maxAngle) || isinf(maxAngle)
				)
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}	
				
				Quaternion attitude;
				if( get_AirframeY_quat( &attitude, TIMEOUT ) == false )
				{
					UnlockCtrl();
					return false;
				}
				
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				
				target_velocity.x = velx;
				target_velocity.y = vely;	
				HorizontalPosition_ControlMode = Position_ControlMode_Velocity;
				VelCtrlMaxRoll = maxAngle;
				if( VelCtrlMaxRoll>0 && VelCtrlMaxRoll<0.1 )
					VelCtrlMaxRoll = 0.1;
				VelCtrlMaxPitch = -1;

				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
			
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( double velx, double vely, double maxRoll, double maxPitch, double TIMEOUT )
		{
			if( isnan(velx) || isinf(velx) ||
					isnan(vely) || isinf(vely) ||
					isnan(maxPitch) || isinf(maxPitch) ||
					isnan(maxRoll) || isinf(maxRoll)
				)
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}	
				
				Quaternion attitude;
				if( get_AirframeY_quat( &attitude, TIMEOUT ) == false )
				{
					UnlockCtrl();
					return false;
				}
				
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				
				double yaw = attitude.getYaw();		
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double velx_ENU = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
				double vely_ENU = BodyHeading2ENU_y( velx , vely , sin_Yaw , cos_Yaw );
				
				target_velocity.x = velx_ENU;
				target_velocity.y = vely_ENU;	
				HorizontalPosition_ControlMode = Position_ControlMode_Velocity;
				VelCtrlMaxRoll = maxRoll;
				if( VelCtrlMaxRoll>0 && VelCtrlMaxRoll<0.1 )
					VelCtrlMaxRoll = 0.1;
				VelCtrlMaxPitch = maxPitch;
				if( VelCtrlMaxPitch>0 && VelCtrlMaxPitch < 0.1 )
					VelCtrlMaxPitch = 0.1;		

				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
			
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_XYLock( double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				if( HorizontalPosition_ControlMode != Position_ControlMode_Position )
					HorizontalPosition_ControlMode = Position_ControlMode_Locking;
				
				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		/*直线飞行*/
			//直线方程系数
			//A=(x1,y1,z1)=target_position目标点
			//B=(x2,y2,z2)=起点
			static vector3<double> route_line_A_B;	//(B-A)
			static double route_line_m;	// 1/(B-A)^2
		
			bool Position_Control_set_TargetPositionXY( double posx, double posy, double TIMEOUT )
			{
				if( isnan(posx) || isinf(posx) ||
						isnan(posy) || isinf(posy)	)
					return false;
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					target_position.x = posx;
					target_position.y = posy;
					vector3<double> position;
					get_Position(&position);
					//calculate vector B-A
					route_line_A_B = position - target_position;
					route_line_A_B.z = 0;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						HorizontalPosition_ControlMode = Position_ControlMode_RouteLine;
					}
					else
						HorizontalPosition_ControlMode = Position_ControlMode_Position;
				
					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_set_TargetPositionXYZ( double posx, double posy, double posz, double TIMEOUT )
			{
				if( isnan(posx) || isinf(posx) ||
						isnan(posy) || isinf(posy) ||
						isnan(posz) || isinf(posz) )
					return false;
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					target_position.x = posx;
					target_position.y = posy;
					target_position.z = posz;
					vector3<double> position;
					get_Position(&position);
					//calculate vector B-A
					route_line_A_B = position - target_position;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_RouteLine3D;
					}
					else
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
				
					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_set_TargetPositionXYRelative( double posx, double posy, double TIMEOUT )
			{
				if( isnan(posx) || isinf(posx) ||
						isnan(posy) || isinf(posy)	)
					return false;
				vector3<double> position;
				get_Position(&position);
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}					
					target_position.x = position.x + posx;
					target_position.y = position.y + posy;				
					//calculate vector B-A
					route_line_A_B = position - target_position;
					route_line_A_B.z = 0;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						HorizontalPosition_ControlMode = Position_ControlMode_RouteLine;
					}
					else
						HorizontalPosition_ControlMode = Position_ControlMode_Position;
				
					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_set_TargetPositionXYZRelative( double posx, double posy, double posz, double TIMEOUT )
			{
				if( isnan(posx) || isinf(posx) ||
						isnan(posy) || isinf(posy) || 
						isnan(posz) || isinf(posz) )
					return false;
				vector3<double> position;
				get_Position(&position);
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}				
					target_position.x = position.x + posx;
					target_position.y = position.y + posy;
					target_position.z = position.z + posz;
					//calculate vector B-A
					route_line_A_B = position - target_position;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_RouteLine3D;
					}
					else
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
				
					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_set_TargetPositionXYRelativeBodyheading( double posx, double posy, double TIMEOUT )
			{
				Quaternion att;
				get_Attitude_quat( &att );
				double yaw = att.getYaw();
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
				double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
				return Position_Control_set_TargetPositionXYRelative( posx_enu, posy_enu, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZRelativeBodyheading( double posx, double posy, double posz, double TIMEOUT )
			{
				Quaternion att;
				get_Attitude_quat( &att );
				double yaw = att.getYaw();
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
				double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
				return Position_Control_set_TargetPositionXYZRelative( posx_enu, posy_enu, posz, TIMEOUT );
			}
			
			bool Position_Control_set_TargetPositionXY_LatLon( double Lat, double Lon, double TIMEOUT )
			{
				if( isnan(Lat) || isinf(Lat) ||
						isnan(Lon) || isinf(Lon)	)
					return false;
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
					return false;
				//获取指定经纬度平面坐标
				double x, y;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				return Position_Control_set_TargetPositionXY( x, y, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZ_LatLon( double Lat, double Lon, double posz, double TIMEOUT )
			{
				if( isnan(Lat) || isinf(Lat) ||
						isnan(Lon) || isinf(Lon) || 
						isnan(posz) || isinf(posz) )
					return false;
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf3 global_inf;
				if( get_OptimalGlobal_XYZ( &global_inf ) == false )
					return false;
				//获取指定经纬度平面坐标
				double x, y;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				posz -= global_inf.HOffset.z;
				return Position_Control_set_TargetPositionXYZ( x, y, posz, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZRA_LatLon( double Lat, double Lon, double posz, double TIMEOUT )
			{
				if( isnan(Lat) || isinf(Lat) ||
						isnan(Lon) || isinf(Lon) || 
						isnan(posz) || isinf(posz) )
					return false;
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
					return false;
				//获取指定经纬度平面坐标
				double x, y;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				//获取起飞位置Z坐标
				double homeZ;
				getHomeLocalZ(&homeZ);
				return Position_Control_set_TargetPositionXYZ( x, y, homeZ + posz, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZRelative_LatLon( double Lat, double Lon, double posz, double TIMEOUT )
			{
				if( isnan(Lat) || isinf(Lat) ||
						isnan(Lon) || isinf(Lon) || 
						isnan(posz) || isinf(posz) )
					return false;
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
					return false;
				//获取指定经纬度平面坐标
				double x, y, z;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				z = global_inf.PositionENU.z + posz;
				return Position_Control_set_TargetPositionXYZ( x, y, z, TIMEOUT );
			}
		/*直线飞行*/
	/*水平位置*/
/*控制接口*/
		
void ctrl_Position()
{	
	bool Attitude_Control_Enabled;	is_Attitude_Control_Enabled(&Attitude_Control_Enabled);
	if( Attitude_Control_Enabled == false )
	{
		Altitude_Control_Enabled = false;
		Position_Control_Enabled = false;
		return;
	}
	
	double e_1_n;
	double e_1;
	double e_2_n;
	double e_2;
	
	bool inFlight;	get_is_inFlight(&inFlight);
	vector3<double> Position;	get_Position_Ctrl(&Position);
	vector3<double> VelocityENU;	get_VelocityENU_Ctrl(&VelocityENU);
	vector3<double> AccelerationENU;	get_AccelerationENU_Ctrl(&AccelerationENU);
	
	//位置速度滤波
	double Ps = 1.0;
	double Pv = 2.0;
	double Pa = 10.0;
	
	static vector3<double> TAcc;
	vector3<double> TargetVelocity;
	vector3<double> TargetVelocity_1;
	vector3<double> TargetVelocity_2;
	
	//XY或Z其中一个为非3D模式则退出3D模式
	if( Is_3DAutoMode(HorizontalPosition_ControlMode) && Is_3DAutoMode(Altitude_ControlMode)==false )
		HorizontalPosition_ControlMode = Position_ControlMode_Locking;
	else if( Is_3DAutoMode(HorizontalPosition_ControlMode)==false && Is_3DAutoMode(Altitude_ControlMode) )
		Altitude_ControlMode = Position_ControlMode_Locking;
	
	if( Position_Control_Enabled )
	{	//水平位置控制
		if( get_Position_MSStatus() != MS_Ready )
		{
			Position_Control_Enabled = false;
			goto PosCtrl_Finish;
		}
		
		switch( HorizontalPosition_ControlMode )
		{
			case Position_ControlMode_Position:
			{
				if( inFlight )
				{
					vector2<double> e1;
					e1.x = target_position.x - Position.x;
					e1.y = target_position.y - Position.y;
					vector2<double> e1_1;
					e1_1.x = - VelocityENU.x;
					e1_1.y = - VelocityENU.y;
					vector2<double> e1_2;
					e1_2.x = - TAcc.x;
					e1_2.y = - TAcc.y;
					double e1_length = safe_sqrt(e1.get_square());
					e_1_n = e1.x*e1_1.x + e1.y*e1_1.y;
					if( !is_zero(e1_length) )
						e_1 = e_1_n / e1_length;
					else
						e_1 = 0;
					e_2_n = ( e1.x*e1_2.x + e1.y*e1_2.y + e1_1.x*e1_1.x + e1_1.y*e1_1.y )*e1_length - e_1*e_1_n;
					if( !is_zero(e1_length*e1_length) )
						e_2 = e_2_n / (e1_length*e1_length);
					else
						e_2 = 0;
					smooth_kp_d2 d1 = smooth_kp_2( e1_length, e_1, e_2 , Ps, 200 );
					vector2<double> T2;
					vector2<double> T2_1;
					vector2<double> T2_2;
					if( !is_zero(e1_length*e1_length*e1_length) )
					{
						vector2<double> n = e1 * (1.0/e1_length);
						vector2<double> n_1 = (e1_1*e1_length - e1*e_1) / (e1_length*e1_length);
						vector2<double> n_2 = ( (e1_2*e1_length-e1*e_2)*e1_length - (e1_1*e1_length-e1*e_1)*(2*e_1) ) / (e1_length*e1_length*e1_length);
						T2 = n*d1.d0;
						T2_1 = n*d1.d1 + n_1*d1.d0;
						T2_2 = n*d1.d2 + n_1*(2*d1.d1) + n_2*d1.d0;
					}
					TargetVelocity.x = T2.x;	TargetVelocity.y = T2.y;
					TargetVelocity_1.x = T2_1.x;	TargetVelocity_1.y = T2_1.y;
					TargetVelocity_2.x = T2_2.x;	TargetVelocity_2.y = T2_2.y;
				}
				else
				{
					//没起飞前在位置控制模式
					//重置期望位置
					target_position.x = Position.x;
					target_position.y = Position.y;
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}		
			case Position_ControlMode_Velocity:
			{
				if( !inFlight )
				{
					//没起飞时重置期望速度
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				else
				{
					TargetVelocity.x = target_velocity.x;
					TargetVelocity.y = target_velocity.y;
					Pa = 20;
				}
				break;
			}
			
			case Position_ControlMode_RouteLine:
			{
				if( inFlight )
				{
					//计算垂足
					vector2<double> A( target_position.x, target_position.y );
					vector2<double> C( Position.x, Position.y );
					vector2<double> A_C = C - A;
					vector2<double> A_B( route_line_A_B.x, route_line_A_B.y );
					double k = (A_C * A_B) * route_line_m;
					vector2<double> foot_point = (A_B * k) + A;
					
					//计算偏差
					vector2<double> e1r = A - foot_point;
					vector2<double> e1d = foot_point - C;
					double e1r_length = safe_sqrt(e1r.get_square());
					double e1d_length = safe_sqrt(e1d.get_square());
					
					//计算route方向单位向量
					vector2<double> route_n;
					if( e1r_length > 0.001 )
						route_n = e1r * (1.0/e1r_length);
					
					//计算d方向单位向量
					vector2<double> d_n;
					if( e1d_length > 0.001 )
						d_n = e1d * (1.0/e1d_length);
					
					//计算e1导数
					vector2<double> e1_1( VelocityENU.x, VelocityENU.y );
					double e1r_1 = -(e1_1 * route_n);
					double e1d_1 = -(e1_1 * d_n);
					//e1二阶导
					vector2<double> e1_2( TAcc.x, TAcc.y );
					double e1r_2 = -(e1_2 * route_n);
					double e1d_2 = -(e1_2 * d_n);
					
					/*route方向*/
						smooth_kp_d2 d1r = smooth_kp_2( e1r_length, e1r_1, e1r_2 , Ps, AutoVelXY );
						vector2<double> T2r = route_n * d1r.d0;
						vector2<double> T2r_1 = route_n * d1r.d1;
						vector2<double> T2r_2 = route_n * d1r.d2;
					/*route方向*/
					
					/*d方向*/
						smooth_kp_d2 d1d = smooth_kp_2( e1d_length, e1d_1, e1d_2 , Ps, AutoVelXY );
						vector2<double> T2d = d_n * d1d.d0;
						vector2<double> T2d_1 = d_n * d1d.d1;
						vector2<double> T2d_2 = d_n * d1d.d2;
					/*d方向*/
						
					TargetVelocity.x = T2r.x+T2d.x;	TargetVelocity.y = T2r.y+T2d.y;
					TargetVelocity_1.x = T2r_1.x+T2d_1.x;	TargetVelocity_1.y = T2r_1.y+T2d_1.y;
					TargetVelocity_2.x = T2r_2.x+T2d_2.x;	TargetVelocity_2.y = T2r_2.y+T2d_2.y;
						
					if( e1r.get_square() + e1d.get_square() < 20*20 )
						HorizontalPosition_ControlMode = Position_ControlMode_Position;
				}
				else
				{
					//没起飞时重置期望速度
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					return;
				}
				break;
			}
			
			case Position_ControlMode_RouteLine3D:
			{
				if( inFlight )
				{
					//计算垂足
					vector3<double> A_C = Position - target_position;
					double k = (A_C * route_line_A_B) * route_line_m;
					vector3<double> foot_point = (route_line_A_B * k) + target_position;
					
					//计算偏差
					vector3<double> e1r = target_position - foot_point;
					vector3<double> e1d = foot_point - Position;
					double e1r_length = safe_sqrt(e1r.get_square());
					double e1d_length = safe_sqrt(e1d.get_square());
					
					//计算route方向单位向量
					vector3<double> route_n;
					if( e1r_length > 0.001 )
						route_n = e1r * (1.0/e1r_length);
					
					//计算e1导数
					double e1r_1_length = -(VelocityENU * route_n);	
					vector3<double> e1r_1 = route_n * e1r_1_length;					
					vector3<double> e1d_1 = -(VelocityENU + e1r_1);
					//e1二阶导
					vector3<double> e1_2( TAcc.x, TAcc.y, TAcc.z );
					double e1r_2_length = -(e1_2 * route_n);
					vector3<double> e1r_2 = route_n * e1r_2_length;					
					vector3<double> e1d_2 = -(e1_2 + e1r_2);
					
					/*route方向*/
						smooth_kp_d2 d1r = smooth_kp_2( e1r_length, e1r_1_length, e1r_2_length , Ps, AutoVelXYZ );
						vector3<double> T2r = route_n * d1r.d0;
						vector3<double> T2r_1 = route_n * d1r.d1;
						vector3<double> T2r_2 = route_n * d1r.d2;
					/*route方向*/
					
					/*d方向*/
						e_1_n = e1d.x*e1d_1.x + e1d.y*e1d_1.y + e1d.z*e1d_1.z;
						if( !is_zero(e1d_length) )
							e_1 = e_1_n / e1d_length;
						else
							e_1 = 0;
						e_2_n = ( e1d.x*e1d_2.x + e1d.y*e1d_2.y + e1d.z*e1d_2.z + e1d_1.x*e1d_1.x + e1d_1.y*e1d_1.y + e1d_1.z*e1d_1.z )*e1d_length - e_1*e_1_n;
						if( !is_zero(e1d_length*e1d_length) )
							e_2 = e_2_n / (e1d_length*e1d_length);
						else
							e_2 = 0;
						smooth_kp_d2 d1d = smooth_kp_2( e1d_length, e_1, e_2 , Ps, AutoVelXYZ );
						vector3<double> T2d;
						vector3<double> T2d_1;
						vector3<double> T2d_2;
						if( !is_zero(e1d_length*e1d_length*e1d_length) )
						{
							vector3<double> n = e1d * (1.0/e1d_length);
							vector3<double> n_1 = (e1d_1*e1d_length - e1d*e_1) / (e1d_length*e1d_length);
							vector3<double> n_2 = ( (e1d_2*e1d_length-e1d*e_2)*e1d_length - (e1d_1*e1d_length-e1d*e_1)*(2*e_1) ) / (e1d_length*e1d_length*e1d_length);
							T2d = n*d1d.d0;
							T2d_1 = n*d1d.d1 + n_1*d1d.d0;
							T2d_2 = n*d1d.d2 + n_1*(2*d1d.d1) + n_2*d1d.d0;
						}
					/*d方向*/
						
					TargetVelocity = T2r + T2d;
					TargetVelocity_1 = T2r_1 + T2d_1;
					TargetVelocity_2 = T2r_2 + T2d_2;
						
					if( e1r.get_square() + e1d.get_square() < 20*20 )
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
				}
				else
				{
					//没起飞时重置期望速度
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					return;
				}
				break;
			}
			
			case Position_ControlMode_Locking:
			default:
			{	//刹车锁位置
				if( inFlight )
				{
					TargetVelocity.x = 0;
					TargetVelocity.y = 0;
					if( VelocityENU.x*VelocityENU.x + VelocityENU.y*VelocityENU.y < 10*10 )
					{
						target_position.x = Position.x;
						target_position.y = Position.y;
						HorizontalPosition_ControlMode = Position_ControlMode_Position;
					}
				}
				else
				{
					target_position.x = Position.x;
					target_position.y = Position.y;
					HorizontalPosition_ControlMode = Position_ControlMode_Position;
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					return;
				}
				break;
			}
		}	
		
		//计算期望加速度
		vector2<double> e2;
		e2.x = TargetVelocity.x - VelocityENU.x;
		e2.y = TargetVelocity.y - VelocityENU.y;
		vector2<double> e2_1;
		e2_1.x = TargetVelocity_1.x - TAcc.x;
		e2_1.y = TargetVelocity_1.y - TAcc.y;
		double e2_length = safe_sqrt(e2.get_square());
		e_1_n = e2.x*e2_1.x + e2.y*e2_1.y;
		if( !is_zero(e2_length) )
			e_1 = e_1_n / e2_length;
		else
			e_1 = 0;
		smooth_kp_d1 d2 = smooth_kp_1( e2_length, e_1 , Pv, 600 );
		vector2<double> T3;
		vector2<double> T3_1;
		if( !is_zero(e2_length*e2_length) )
		{
			vector2<double> n = e2 * (1.0/e2_length);
			vector2<double> n_1 = (e2_1*e2_length - e2*e_1) / (e2_length*e2_length);
			T3 = n*d2.d0;
			T3_1 = n*d2.d1 + n_1*d2.d0;
		}
		T3 += vector2<double>( TargetVelocity_1.x, TargetVelocity_1.y );
		T3_1 += vector2<double>( TargetVelocity_2.x, TargetVelocity_2.y );
		
		vector2<double> e3;
		e3.x = T3.x - TAcc.x;
		e3.y = T3.y - TAcc.y;
		double e3_length = safe_sqrt(e3.get_square());
		double d3 = smooth_kp_0( e3_length , Pa, 6000 );
		vector2<double> T4;
		if( !is_zero(e3_length) )
		{
			vector2<double> n = e3 * (1.0/e3_length);
			T4 = n*d3;
		}
		T4 += T3_1;
		
		TAcc.x += T4.x*(1.0/CtrlRateHz);
		TAcc.y += T4.y*(1.0/CtrlRateHz);	
		
		//去除风力扰动
		vector3<double> WindDisturbance;
		get_WindDisturbance( &WindDisturbance );
		vector2<double> target_acceleration;
		target_acceleration.x = TAcc.x - WindDisturbance.x;
		target_acceleration.y = TAcc.y - WindDisturbance.y;
		
		//旋转至Bodyheading
		Quaternion attitude;
		get_Attitude_quat(&attitude);
		double yaw = attitude.getYaw();		
		double sin_Yaw, cos_Yaw;
		fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
		double target_acceleration_x_bodyheading = ENU2BodyHeading_x( target_acceleration.x , target_acceleration.y , sin_Yaw , cos_Yaw );
		double target_acceleration_y_bodyheading = ENU2BodyHeading_y( target_acceleration.x , target_acceleration.y , sin_Yaw , cos_Yaw );
		
		//计算风力补偿角度
		double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
		double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
		double AntiDisturbancePitch = atan2( -WindDisturbance_Bodyheading_x , GravityAcc );
		double AntiDisturbanceRoll = atan2( WindDisturbance_Bodyheading_y , GravityAcc );
		
		//计算目标角度
		double target_Roll = atan2( -target_acceleration_y_bodyheading , GravityAcc );
		double target_Pitch = atan2( target_acceleration_x_bodyheading , GravityAcc );
		if( HorizontalPosition_ControlMode==Position_ControlMode_Velocity )
		{	//角度限幅
			if( VelCtrlMaxRoll>0 && VelCtrlMaxPitch>0 )
			{
				target_Roll = constrain( target_Roll , AntiDisturbanceRoll - VelCtrlMaxRoll, AntiDisturbanceRoll + VelCtrlMaxRoll );
				target_Pitch = constrain( target_Pitch , AntiDisturbancePitch - VelCtrlMaxPitch, AntiDisturbancePitch + VelCtrlMaxPitch );
			}
			else if( VelCtrlMaxRoll>0 )
			{
				vector2<double> Tangle( target_Roll - AntiDisturbanceRoll, target_Pitch - AntiDisturbancePitch );
				Tangle.constrain(VelCtrlMaxRoll);
				target_Roll = AntiDisturbanceRoll + Tangle.x;
				target_Pitch = AntiDisturbancePitch + Tangle.y;
			}
		}

		//设定目标角度
		Attitude_Control_set_Target_RollPitch( target_Roll, target_Pitch );
		
		//获取真实目标角度修正TAcc
		Attitude_Control_get_Target_RollPitch( &target_Roll, &target_Pitch );
		target_acceleration_x_bodyheading = tan(target_Pitch)*GravityAcc;
		target_acceleration_y_bodyheading = -tan(target_Roll)*GravityAcc;
		target_acceleration.x = BodyHeading2ENU_x( target_acceleration_x_bodyheading, target_acceleration_y_bodyheading , sin_Yaw, cos_Yaw );
		target_acceleration.y = BodyHeading2ENU_y( target_acceleration_x_bodyheading, target_acceleration_y_bodyheading , sin_Yaw, cos_Yaw );
		TAcc.x = target_acceleration.x + WindDisturbance.x;
		TAcc.y = target_acceleration.y + WindDisturbance.y;
	}//水平位置控制
	
PosCtrl_Finish:	
	if( Altitude_Control_Enabled )
	{//高度控制
			
		if( !Is_3DAutoMode(Altitude_ControlMode) )
		{
			switch( Altitude_ControlMode )
			{
				case Position_ControlMode_Position:
				{	//控制位置
					if( inFlight )
					{
						Target_tracker[2].r2n = Target_tracker[2].r2p = 200;
						Target_tracker[2].track4( target_position.z , 1.0 / CtrlRateHz );
					}
					else
					{
						//没起飞前在位置控制模式
						//不要起飞
						Target_tracker[2].reset();
						target_position.z = Target_tracker[2].x1 = Position.z;
						Attitude_Control_set_Throttle( get_STThrottle() );
						goto AltCtrl_Finish;
					}
					break;
				}
				case Position_ControlMode_Velocity:
				{	//控制速度
					if( inFlight || target_velocity.z > 0 )
						Target_tracker[2].track3( target_velocity.z , 1.0 / CtrlRateHz );
					else
					{
						//没起飞且期望速度为负
						//不要起飞
						Target_tracker[2].reset();
						Target_tracker[2].x1 = Position.z;
						Attitude_Control_set_Throttle( get_STThrottle() );
						goto AltCtrl_Finish;
					}
					break;
				}
				
				case Position_ControlMode_Takeoff:
				{	//起飞
					if( inFlight )
					{
						//已起飞
						//控制达到目标高度
						double homeZ;
						getHomeLocalZ(&homeZ);
						if( Position.z - homeZ < 100 )
							Target_tracker[2].r2n = Target_tracker[2].r2p = 50;
						else
							Target_tracker[2].r2n = Target_tracker[2].r2p = AutoVelZ;
						Target_tracker[2].track4( target_position.z , 1.0 / CtrlRateHz );
						if( fabs( target_position.z - Position.z ) < 10 && \
								in_symmetry_range( VelocityENU.z , 10.0 ) &&  \
								in_symmetry_range( AccelerationENU.z , 50.0 ) && \
								in_symmetry_range( Target_tracker[2].x2 , 0.1 ) && \
								in_symmetry_range( Target_tracker[2].x3 , 0.1 )	)
							Altitude_ControlMode = Position_ControlMode_Position;
					}
					else
					{
						//未起飞
						//等待起飞
						target_position.z =  Position.z + TakeoffHeight;
						Target_tracker[2].x1 = Position.z;
						Target_tracker[2].track3( 50 , 1.0 / CtrlRateHz );
					}
					break;
				}
				case Position_ControlMode_RouteLine:
				{	//飞到指定高度
					if( inFlight )
					{
						//已起飞
						//控制达到目标高度
						Target_tracker[2].r2n = Target_tracker[2].r2p = AutoVelZ;
						Target_tracker[2].track4( target_position.z , 1.0f / CtrlRateHz );
						if( fabs( target_position.z - Position.z ) < 10 && \
								in_symmetry_range( VelocityENU.z , 10.0f ) &&  \
								in_symmetry_range( AccelerationENU.z , 50.0f ) && \
								in_symmetry_range( Target_tracker[2].x2 , 0.1f ) && \
								in_symmetry_range( Target_tracker[2].x3 , 0.1f )	)
							Altitude_ControlMode = Position_ControlMode_Position;
					}
					else
					{
						//未起飞
						//不要起飞
						Target_tracker[2].reset();
						Target_tracker[2].x1 = Position.z;
						Attitude_Control_set_Throttle( 0 );
						goto AltCtrl_Finish;
					}
					break;
				}
				
				case Position_ControlMode_Locking:
				default:
				{	//锁位置（减速到0然后锁住高度）
					if( inFlight )
					{
						Target_tracker[2].track3( 0 , 1.0 / CtrlRateHz );
						if( in_symmetry_range( VelocityENU.z , 10.0 ) && in_symmetry_range( AccelerationENU.z , 50.0 ) && \
								in_symmetry_range( Target_tracker[2].x2 , 0.1 ) && \
								in_symmetry_range( Target_tracker[2].x3 , 0.1 )	)
						{
							target_position.z = Target_tracker[2].x1 = Position.z;
							Altitude_ControlMode = Position_ControlMode_Position;
						}
					}
					else
					{
						Altitude_ControlMode = Position_ControlMode_Position;
						Attitude_Control_set_Throttle( get_STThrottle() );
						goto AltCtrl_Finish;
					}
					break;
				}
			}
		}
		
		if( inFlight )
		{
			//计算期望速度
			double target_velocity_z;
			//期望垂直速度的导数
			double Tvz_1;
			if( Is_3DAutoMode(Altitude_ControlMode) )
			{
				target_velocity_z = TargetVelocity.z;
				Tvz_1 = TargetVelocity_1.z;			
				Target_tracker[2].reset();
				Target_tracker[2].x1 = target_position.z;
			}
			else
			{
				if( Target_tracker[2].get_tracking_mode() == 4 )
				{
					target_velocity_z = Ps * ( Target_tracker[2].x1 - Position.z ) + Target_tracker[2].x2;
					Tvz_1 = Ps * ( Target_tracker[2].x2 - VelocityENU.z ) + Target_tracker[2].x3;
				}
				else
				{
					target_velocity_z = Target_tracker[2].x2;
					Tvz_1 = Target_tracker[2].x3;
				}
			}
			
			//计算期望加速度
			double target_acceleration_z = Pv * ( target_velocity_z - VelocityENU.z ) + Tvz_1;
			target_acceleration_z = TargetVelocityFilter[2].run( target_acceleration_z );
			//加速度误差
			double acceleration_z_error = target_acceleration_z - AccelerationENU.z;
			
			//获取倾角cosin
			Quaternion quat;
			get_Airframe_quat(&quat);
			double lean_cosin = quat.get_lean_angle_cosin();
			if( lean_cosin < 0.5 )
				lean_cosin = 0.5;
			
			//获取电机起转油门
			double MotorStartThrottle = get_STThrottle();
			//获取悬停油门 - 电机起转油门
			double hover_throttle;
			get_hover_throttle(&hover_throttle);
			hover_throttle = hover_throttle - MotorStartThrottle;
			
			//悬停油门滤波
			HoverThrottleFilter.track4( hover_throttle, 1.0/CtrlRateHz, 2, 2, 2, 2 );
			//自动计算高度控制D
			double ZD = 0.0005*HoverThrottleFilter.get_x1();			
			
			//悬停油门时：
			//输出力 = mg
			//需获得Ta加速度时：
			//需输出的力为：( 1 + Ta/G )
			//计算输出油门 =    悬停油门 * ( 1 + Ta / G )  +   P*( Ta - a )
			double throttle = hover_throttle * ( 1.0f + target_acceleration_z / GravityAcc ) + ZD*acceleration_z_error;
			//倾角补偿
			throttle /= lean_cosin;
			
			//油门限幅
			throttle += MotorStartThrottle;
			if( throttle > 90 )
				throttle = 90;
			if( inFlight )
			{
				if( throttle < MotorStartThrottle )
					throttle = MotorStartThrottle;
			}
			
			//输出
			Attitude_Control_set_Throttle( throttle );
		}
		else
		{
			//没起飞
			//均匀增加油门起飞
			double throttle;
			get_Target_Throttle(&throttle);
			Attitude_Control_set_Throttle( throttle + 1.0/CtrlRateHz * 15 );
		}
		
	}//高度控制
AltCtrl_Finish:
	return;
}

void init_Ctrl_Position()
{
	//初始化期望TD4滤波器
	Target_tracker[0] = TD4( 15 , 15 , 15 , 15 );
	Target_tracker[1] = TD4( 15 , 15 , 15 , 15 );
	Target_tracker[2] = TD4( 2 , 5 , 50 , 60 );	
	Target_tracker[2].r3p = 900;
	Target_tracker[2].r3n = 600;
	
	//初始化期望速度低通滤波器
	TargetVelocityFilter[0].set_cutoff_frequency( CtrlRateHz , 10 );
	TargetVelocityFilter[1].set_cutoff_frequency( CtrlRateHz , 10 );
	TargetVelocityFilter[2].set_cutoff_frequency( CtrlRateHz , 10 );
	
	PosCtrlCfg initial_cfg;
	//XY航线速度
	initial_cfg.AutoVXY[0] = 500;
	//Z航线速度
	initial_cfg.AutoVZ[0] = 200;
	//XYZ航线速度
	initial_cfg.AutoVXYZ[0] = 500;	
	//高度前馈滤波器
	initial_cfg.Z_TD4P1[0] = 2;
	initial_cfg.Z_TD4P2[0] = 5;
	initial_cfg.Z_TD4P3[0] = 50;
	initial_cfg.Z_TD4P4[0] = 60;

	MAV_PARAM_TYPE param_types[] = {
		//XY航线速度
		MAV_PARAM_TYPE_REAL32 ,	
		//Z航线速度
		MAV_PARAM_TYPE_REAL32 ,	
		//XYZ航线速度
		MAV_PARAM_TYPE_REAL32 ,	
		//高度前馈滤波器
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
	};

	SName param_names[] = {
		//XY航线速度
		"PC_AutoVXY" ,	
		//Z航线速度
		"PC_AutoVZ" ,	
		//XYZ航线速度
		"PC_AutoVXYZ" ,	
		//高度前馈滤波器
		"PC_Z_TD4P1" ,
		"PC_Z_TD4P2" ,
		"PC_Z_TD4P3" ,
		"PC_Z_TD4P4" ,
	};
	ParamGroupRegister( "PosCtrl", 1, 7, param_types, param_names, (uint64_t*)&initial_cfg );
}