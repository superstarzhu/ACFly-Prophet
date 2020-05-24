#include "NavCmdProcess.hpp"
#include "Basic.hpp"
#include "MeasurementSystem.hpp"
#include "ControlSystem.hpp"
#include "mavlink.h"

/*
	Nav飞行控制指令处理
	所有指令必须在水平位置控制器打开的前提下执行
	所有参数单位角度为度，距离速度为米
	参数：
		freq：运行频率
		params：7个参数数组
		counter1、counter2：两个累加器，开始和完成后都应是0
	返回：
		<-2：错误
		-2：未完成
		-1：完成
		>=0：完成且要求切换到指定Mission
*/



/*NavCmd16_WAYPOINT
	MAV_CMD_NAV_WAYPOINT
	航点飞行（调转机头并飞行到指定点）
	参数:
		<description>Navigate to waypoint.</description>
		<param index="0" label="Hold" units="s" minValue="0">Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)</param>
		<param index="1" label="Accept Radius" units="m" minValue="0">Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)</param>
		<param index="2" label="Pass Radius" units="m">0 to pass through the WP, if &gt; 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.</param>
		<param index="3" label="Yaw" units="deg">Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).</param>
		<param index="4" label="Latitude">Latitude</param>
		<param index="5" label="Longitude">Longitude</param>
		<param index="6" label="Altitude" units="m">Altitude</param>
*/
static int16_t NavCmd16_WAYPOINT( double freq, uint8_t frame, double params[], NavCmdInf* inf )
{	
	if( get_Position_MSStatus() != MS_Ready )
	{	//无定位出错
		inf->counter1 = inf->counter2 = 0;
		return -3;
	}
	bool inFlight;
	get_is_inFlight(&inFlight);
	if( inFlight == false )
	{	//未起飞出错
		inf->counter1 = inf->counter2 = 0;
		return -3;
	}
	
	switch( inf->counter1 )
	{
		case 0:
		{	//判断执行旋转偏航
			if( isnan(params[3]) || isinf(params[3]) )
			{	//机头指向航点方向
				Position_Control_set_XYLock();
				Position_Control_set_ZLock();
				
				double LA, LB;
				switch(frame)
				{
					case MAV_FRAME_GLOBAL:
					case MAV_FRAME_GLOBAL_RELATIVE_ALT:
					case MAV_FRAME_GLOBAL_INT:
					case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
					case MAV_FRAME_GLOBAL_TERRAIN_ALT:
					case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
					{	//全球定位
						if( params[4]<-90 ||  params[4]> 90 
							|| params[5]<-180 || params[5]>180 )
						{	//经纬度为不正确不转偏航
							inf->counter1 = 1;
							inf->counter2 = freq*3;
							break;
						}
						
						//获取最优全球定位传感器信息
						PosSensorHealthInf2 global_inf;
						if( get_OptimalGlobal_XY( &global_inf ) == false )
						{
							inf->counter1 = inf->counter2 = 0;
							return -3;
						}
						//获取指定经纬度平面坐标
						double x, y;
						map_projection_project( &global_inf.mp, params[4], params[5], &x, &y );
						x -= global_inf.HOffset.x;
						y -= global_inf.HOffset.y;
						LA = y - global_inf.PositionENU.y;
						LB = x - global_inf.PositionENU.x;
						break;
					}
					
					case MAV_FRAME_LOCAL_NED:
					{
						vector3<double> position;
						get_Position(&position);
						LA = params[4] - position.y;
						LB = params[5] - position.x;
						break;
					}
					
					case MAV_FRAME_LOCAL_ENU:
					{
						vector3<double> position;
						get_Position(&position);
						LA = params[5] - position.y;
						LB = params[4] - position.x;
						break;
					}
					
					case MAV_FRAME_LOCAL_OFFSET_NED:
					{
						LA = params[4];
						LB = params[5];
						break;
					}
					
					case MAV_FRAME_BODY_NED:
					case MAV_FRAME_BODY_FRD:
					case MAV_FRAME_BODY_OFFSET_NED:
					{
						Quaternion att;
						get_Attitude_quat( &att );
						double yaw = att.getYaw();
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						double posx_enu = BodyHeading2ENU_x( params[5] , params[4] , sin_Yaw , cos_Yaw );
						double posy_enu = BodyHeading2ENU_y( params[5] , params[4] , sin_Yaw , cos_Yaw );
						LA = posy_enu;
						LB = posx_enu;
						break;
					}
					
					case MAV_FRAME_BODY_FLU:
					{
						Quaternion att;
						get_Attitude_quat( &att );
						double yaw = att.getYaw();
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						double posx_enu = BodyHeading2ENU_x( params[4] , params[5] , sin_Yaw , cos_Yaw );
						double posy_enu = BodyHeading2ENU_y( params[4] , params[5] , sin_Yaw , cos_Yaw );
						LA = posy_enu;
						LB = posx_enu;
						break;
					}
					
					default:
						inf->counter1 = inf->counter2 = 0;
						return -3;
				}
				
				Attitude_Control_set_Target_Yaw( atan2(LA,LB) );
				inf->counter1 = 1;
				inf->counter2 = 0;
			}
			else if( params[3]>-360 && params[3]<360 )
			{	//指定偏航朝向
				Attitude_Control_set_Target_Yaw( degree2rad(params[3]) );
				inf->counter1 = 1;
				inf->counter2 = 0;
			}
			else
			{	//不旋转偏航
				inf->counter1 = 1;
				inf->counter2 = freq*3;
			}
			break;
		}
		
		case 1:
		{	//等待偏航旋转开始航点飞行
			Position_Control_set_XYLock();
			Position_Control_set_ZLock();
			if( ++inf->counter2 >= freq*3 )
			{	
				bool res;
				switch(frame)
				{
					case MAV_FRAME_GLOBAL_INT:
					case MAV_FRAME_GLOBAL:
					{
						if( params[4]<-90 ||  params[4]> 90 
							|| params[5]<-180 || params[5]>180 )
						{	//经纬度不正确只进行高度调整
							Position_Control_set_XYLock();
							res = Position_Control_set_TargetPositionZGlobal( params[6]*100 );
						}
						else	//经纬度正确进行三维飞行
							res = Position_Control_set_TargetPositionXYZ_LatLon( params[4], params[5], params[6]*100 );
						break;
					}
					case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
					case MAV_FRAME_GLOBAL_RELATIVE_ALT:
					{
						if( params[4]<-90 ||  params[4]> 90 
							|| params[5]<-180 || params[5]>180 )
						{	//经纬度不正确只进行高度调整
							Position_Control_set_XYLock();
							res = Position_Control_set_TargetPositionZRA( params[6]*100 );
						}
						else	//经纬度正确进行三维飞行
							res = Position_Control_set_TargetPositionXYZRA_LatLon( params[4], params[5], params[6]*100 );
						break;
					}
					case MAV_FRAME_GLOBAL_TERRAIN_ALT:
					case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
					{
						if( params[4]<-90 ||  params[4]> 90 
							|| params[5]<-180 || params[5]>180 )
						{	//经纬度不正确只进行高度调整
							Position_Control_set_XYLock();
							res = Position_Control_set_TargetPositionZRA( params[6]*100 );
						}
						else	//经纬度正确进行三维飞行
							res = Position_Control_set_TargetPositionXYZRA_LatLon( params[4], params[5], params[6]*100 );
						break;
					}
					
					case MAV_FRAME_LOCAL_NED:
						res = Position_Control_set_TargetPositionXYZ( params[5]*100, params[4]*100, params[6]*100 );
						break;
					
					case MAV_FRAME_LOCAL_ENU:
						res = Position_Control_set_TargetPositionXYZ( params[4]*100, params[5]*100, params[6]*100 );
						break;
					
					case MAV_FRAME_LOCAL_OFFSET_NED:
						res = Position_Control_set_TargetPositionXYZRelative( params[5]*100, params[4]*100, params[6]*100 );
						break;
					
					case MAV_FRAME_BODY_NED:
					case MAV_FRAME_BODY_FRD:
					case MAV_FRAME_BODY_OFFSET_NED:
					{
						//body系修正航向后只向前走
						if( isnan(params[3]) || isinf(params[3]) )
							res = Position_Control_set_TargetPositionXYZRelativeBodyheading( safe_sqrt(params[5]*params[5] + params[4]*params[4])*100, 0, params[6]*100 );
						else
							res = Position_Control_set_TargetPositionXYZRelativeBodyheading( params[5]*100, params[4]*100, params[6]*100 );
						break;
					}
					
					case MAV_FRAME_BODY_FLU:
					{
						//body系修正航向后只向前走
						if( isnan(params[3]) || isinf(params[3]) )
							res = Position_Control_set_TargetPositionXYZRelativeBodyheading( safe_sqrt(params[4]*params[4] + params[5]*params[5])*100, 0, params[6]*100 );
						else
							res = Position_Control_set_TargetPositionXYZRelativeBodyheading( params[4]*100, params[5]*100, params[6]*100 );
						break;
					}
					
					default:
						inf->counter1 = inf->counter2 = 0;
						return -3;
				}
				
				if(res)
				{	//成功
					inf->counter1 = 2;
					inf->counter2 = 0;
				}
				else
				{	//失败
					inf->counter1 = 0;
					inf->counter2 = 0;
					return -3;
				}
			}
			break;
		}
		
		case 2:
		{	//等待航点飞行完成
			Position_ControlMode alt_mode, pos_mode;
			get_Altitude_ControlMode(&alt_mode);
			get_Position_ControlMode(&pos_mode);
			if( alt_mode == Position_ControlMode_Position )
				Position_Control_set_ZLock();
			if( pos_mode == Position_ControlMode_Position )
				Position_Control_set_XYLock();
			if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
			{	//飞行完成进入hold等待
				inf->counter1 = 3;
				inf->counter2 = 0;
			}
			break;
		}
		
		case 3:
		{	//hold
			Position_Control_set_XYLock();
			Position_Control_set_ZLock();
			if( ++(inf->counter2) >= freq*params[0] )
			{
				inf->counter1 = 0;
				inf->counter2 = 0;
				return -1;
			}
			break;
		}
	}
	return -2;
}

/*NavCmd20_RETURN_TO_LAUNCH
	MAV_CMD_NAV_RETURN_TO_LAUNCH
	回到起飞点
	参数:
		<description>Return to launch location</description>
		<param index="1">Empty</param>
		<param index="2">Empty</param>
		<param index="3">Empty</param>
		<param index="4">Empty</param>
		<param index="5">Empty</param>
		<param index="6">Empty</param>
		<param index="7">Empty</param>
*/
static int16_t NavCmd20_RETURN_TO_LAUNCH( double freq, uint8_t frame, double params[], NavCmdInf* inf )
{	
	if( get_Position_MSStatus() != MS_Ready )
	{	//无定位出错
		inf->counter1 = inf->counter2 = 0;
		return -3;
	}
	bool inFlight;
	get_is_inFlight(&inFlight);
	if( inFlight == false )
	{	//未起飞出错
		inf->counter1 = inf->counter2 = 0;
		return -3;
	}
	
	switch( inf->counter1 )
	{
		case 0:
		{	//旋转偏航
			Position_Control_set_XYLock();
			Position_Control_set_ZLock();
			
			double LA, LB;
			
			vector2<double> homeP;
			if( getHomeLatLon(&homeP) )
			{	//返航至经纬度
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
				{
					goto TurnYawLocalP;
				}
				//获取指定经纬度平面坐标
				double x, y;
				map_projection_project( &global_inf.mp, homeP.x, homeP.y, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				LA = y - global_inf.PositionENU.y;
				LB = x - global_inf.PositionENU.x;
			}
			else if( getHomePoint(&homeP) )
			{	//返航至Local坐标
TurnYawLocalP:
				vector3<double> position;
				get_Position(&position);
				LA = params[5] - position.y;
				LB = params[4] - position.x;
			}
			else
			{	//无返航点
				inf->counter1 = inf->counter2 = 0;
				return -3;
			}
			
			Attitude_Control_set_Target_Yaw( atan2(LA,LB) );
			inf->counter1 = 1;
			inf->counter2 = 0;
			
			break;
		}
		
		case 1:
		{	//等待偏航旋转开始航点飞行
			Position_Control_set_XYLock();
			Position_Control_set_ZLock();
			if( ++inf->counter2 >= freq*3 )
			{	
				bool res;
				
				vector2<double> homeP;
				if( getHomeLatLon(&homeP) )
				{	//返航至经纬度
					res = Position_Control_set_TargetPositionXY_LatLon( homeP.x, homeP.y );
					if( res == false )
						goto FlyLocalP;
				}
				else if( getHomePoint(&homeP) )
				{	//返航至local坐标
FlyLocalP:
					res = Position_Control_set_TargetPositionXY( homeP.x, homeP.y );
				}
				else
				{	//无返航点
					inf->counter1 = inf->counter2 = 0;
					return -3;
				}
				
				if(res)
				{	//成功
					inf->counter1 = 2;
					inf->counter2 = 0;
				}
				else
				{	//失败
					inf->counter1 = 0;
					inf->counter2 = 0;
					return -3;
				}
			}
			break;
		}
		
		case 2:
		{	//等待航点飞行完成
			Position_Control_set_ZLock();
			Position_ControlMode pos_mode;
			get_Position_ControlMode(&pos_mode);
			if( pos_mode==Position_ControlMode_Position )
			{	//飞行完成
				inf->counter1 = 0;
				inf->counter2 = 0;
				return -1;
			}
			break;
		}
	}
	return -2;
}

/*NavCmd22_TAKEOFF
	MAV_CMD_NAV_TAKEOFF
	起飞并飞往指定地点
	参数:
		<description>Takeoff from ground / hand</description>
		<param index="0" label="Pitch">Minimum pitch (if airspeed sensor present), desired pitch without sensor</param>
		<param index="1">Empty</param>
		<param index="2">Empty</param>
		<param index="3" label="Yaw" units="deg">Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).</param>
		<param index="4" label="Latitude">Latitude</param>
		<param index="5" label="Longitude">Longitude</param>
		<param index="6" label="Altitude" units="m">Altitude</param>
*/
static int16_t NavCmd22_TAKEOFF( double freq, uint8_t frame, double params[], NavCmdInf* inf )
{
	if( get_Position_MSStatus() != MS_Ready )
	{	//无定位出错
		inf->counter1 = inf->counter2 = 0;
		return -3;
	}
	
	switch( inf->counter1 )
	{
		case 0:
		{	//起飞到指定高度
			bool res;
			switch(frame)
			{
				case MAV_FRAME_GLOBAL_INT:
				case MAV_FRAME_GLOBAL:
					res = Position_Control_Takeoff_HeightGlobal( params[6]*100 );
					break;
				
				case MAV_FRAME_LOCAL_OFFSET_NED:
				case MAV_FRAME_BODY_NED:
				case MAV_FRAME_BODY_FRD:
				case MAV_FRAME_BODY_OFFSET_NED:
				case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
				case MAV_FRAME_GLOBAL_RELATIVE_ALT:
				case MAV_FRAME_GLOBAL_TERRAIN_ALT:
				case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
				case MAV_FRAME_BODY_FLU:
					res = Position_Control_Takeoff_HeightRelative( params[6]*100 );
					break;
				
				case MAV_FRAME_LOCAL_NED:
				case MAV_FRAME_LOCAL_ENU:
					res = Position_Control_Takeoff_Height( params[6]*100 );
					break;
				
				default:
					inf->counter1 = inf->counter2 = 0;
					return -3;
			}

			if(res)
			{	//成功
				inf->counter1 = 1;
				inf->counter2 = 0;
			}
			else
			{	//失败
				inf->counter1 = 0;
				inf->counter2 = 0;
				return -3;
			}
			break;
		}
		
		case 1:
		{	//等待起飞完成旋转偏航
			Position_ControlMode mode;
			Position_Control_set_XYLock();
			get_Altitude_ControlMode(&mode);
			if( mode == Position_ControlMode_Position )
			{	//判断执行旋转偏航			
				if( isnan(params[3]) || isinf(params[3]) )
				{	//机头指向航点方向
					Position_Control_set_XYLock();
					Position_Control_set_ZLock();
					
					double LA, LB;
					switch(frame)
					{
						case MAV_FRAME_GLOBAL:
						case MAV_FRAME_GLOBAL_RELATIVE_ALT:
						case MAV_FRAME_GLOBAL_INT:
						case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
						case MAV_FRAME_GLOBAL_TERRAIN_ALT:
						case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
						{	//全球定位
							if( params[4]<-90 ||  params[4]> 90 
								|| params[5]<-180 || params[5]>180 )
							{	//经纬度为不正确不转偏航
								inf->counter1 = 2;
								inf->counter2 = freq*3;
								break;
							}
							
							//获取最优全球定位传感器信息
							PosSensorHealthInf2 global_inf;
							if( get_OptimalGlobal_XY( &global_inf ) == false )
							{
								inf->counter1 = inf->counter2 = 0;
								return -3;
							}
							//获取指定经纬度平面坐标
							double x, y;
							map_projection_project( &global_inf.mp, params[4], params[5], &x, &y );
							x -= global_inf.HOffset.x;
							y -= global_inf.HOffset.y;
							LA = y - global_inf.PositionENU.y;
							LB = x - global_inf.PositionENU.x;
							break;
						}
						
						case MAV_FRAME_LOCAL_NED:
						{
							vector3<double> position;
							get_Position(&position);
							LA = params[4] - position.y;
							LB = params[5] - position.x;
							break;
						}
						
						case MAV_FRAME_LOCAL_ENU:
						{
							vector3<double> position;
							get_Position(&position);
							LA = params[5] - position.y;
							LB = params[4] - position.x;
							break;
						}
						
						case MAV_FRAME_LOCAL_OFFSET_NED:
						{
							LA = params[4];
							LB = params[5];
							break;
						}
						
						case MAV_FRAME_BODY_NED:
						case MAV_FRAME_BODY_FRD:
						case MAV_FRAME_BODY_OFFSET_NED:
						{
							Quaternion att;
							get_Attitude_quat( &att );
							double yaw = att.getYaw();
							double sin_Yaw, cos_Yaw;
							fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
							double posx_enu = BodyHeading2ENU_x( params[5] , params[4] , sin_Yaw , cos_Yaw );
							double posy_enu = BodyHeading2ENU_y( params[5] , params[4] , sin_Yaw , cos_Yaw );
							LA = posy_enu;
							LB = posx_enu;
							break;
						}
						
						case MAV_FRAME_BODY_FLU:
						{
							Quaternion att;
							get_Attitude_quat( &att );
							double yaw = att.getYaw();
							double sin_Yaw, cos_Yaw;
							fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
							double posx_enu = BodyHeading2ENU_x( params[4] , params[5] , sin_Yaw , cos_Yaw );
							double posy_enu = BodyHeading2ENU_y( params[4] , params[5] , sin_Yaw , cos_Yaw );
							LA = posy_enu;
							LB = posx_enu;
							break;
						}
						
						default:
							inf->counter1 = inf->counter2 = 0;
							return -3;
					}
					
					if( LA==0 && LB==0 )
					{	//无距离退出
						inf->counter1 = 0;
						inf->counter2 = 0;
						return -1;
					}
					Attitude_Control_set_Target_Yaw( atan2(LA,LB) );
					inf->counter1 = 2;
					inf->counter2 = 0;
				}
				else if( params[3]>-360 && params[3]<360 )
				{	//指定偏航朝向
					Attitude_Control_set_Target_Yaw( degree2rad(params[3]) );
					inf->counter1 = 2;
					inf->counter2 = 0;
				}
				else
				{	//不旋转偏航
					inf->counter1 = 2;
					inf->counter2 = freq*3;
				}
			}
			break;
		}
		
		case 2:
		{	//等待偏航旋转开始航点飞行
			Position_Control_set_XYLock();
			Position_Control_set_ZLock();
			if( ++inf->counter2 >= freq*3 )
			{	//飞直线
				bool res;
				switch(frame)
				{
					case MAV_FRAME_GLOBAL_INT:
					case MAV_FRAME_GLOBAL:
					case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
					case MAV_FRAME_GLOBAL_RELATIVE_ALT:
					case MAV_FRAME_GLOBAL_TERRAIN_ALT:
					case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
					{
						if( params[4]<-90 ||  params[4]> 90 
							|| params[5]<-180 || params[5]>180 )
						{	//经纬度不正确结束
							inf->counter1 = 0;
							inf->counter2 = 0;
							return -1;
						}
						res = Position_Control_set_TargetPositionXY_LatLon( params[4], params[5] );
						break;
					}
					
					case MAV_FRAME_LOCAL_NED:
						res = Position_Control_set_TargetPositionXY( params[5]*100, params[4]*100 );
						break;
					
					case MAV_FRAME_LOCAL_ENU:
						res = Position_Control_set_TargetPositionXY( params[4]*100, params[5]*100 );
						break;
					
					case MAV_FRAME_LOCAL_OFFSET_NED:
						res = Position_Control_set_TargetPositionXYRelative( params[5]*100, params[4]*100 );
						break;
					
					case MAV_FRAME_BODY_NED:
					case MAV_FRAME_BODY_FRD:
					case MAV_FRAME_BODY_OFFSET_NED:
					{
						//body系修正航向后只向前走
						if( isnan(params[3]) || isinf(params[3]) )
							res = Position_Control_set_TargetPositionXYZRelativeBodyheading( safe_sqrt(params[4]*params[4] + params[5]*params[5])*100, 0, params[6]*100 );
						else
							res = Position_Control_set_TargetPositionXYRelativeBodyheading( params[5]*100, params[4]*100 );
						break;
					}
					
					case MAV_FRAME_BODY_FLU:
					{
						//body系修正航向后只向前走
						if( isnan(params[3]) || isinf(params[3]) )
							res = Position_Control_set_TargetPositionXYZRelativeBodyheading( safe_sqrt(params[4]*params[4] + params[5]*params[5])*100, 0, params[6]*100 );
						else
							res = Position_Control_set_TargetPositionXYRelativeBodyheading( params[4]*100, params[5]*100 );
						break;
					}
					
					default:
						inf->counter1 = inf->counter2 = 0;
						return -3;
				}
				
				Position_Control_set_ZLock();
				if(res)
				{	//成功
					inf->counter1 = 3;
					inf->counter2 = 0;
				}
				else
				{	//失败
					inf->counter1 = 0;
					inf->counter2 = 0;
					return -3;
				}
			}
			break;
		}
		
		case 3:
		{	//等待飞直线完成			
			Position_Control_set_ZLock();
			Position_ControlMode pos_mode;
			get_Position_ControlMode(&pos_mode);
			if( pos_mode == Position_ControlMode_Position )
				Position_Control_set_XYLock();
			if( pos_mode==Position_ControlMode_Position )
			{	//飞行完成
				inf->counter1 = 0;
				inf->counter2 = 0;
				return -1;
			}
			break;
		}
	}
	return -2;
}

/*NavCmd93_DELAY
	MAV_CMD_NAV_DELAY
	延时
	参数:
		<description>Delay the next navigation command a number of seconds or until a specified time</description>
		<param index="1" label="Delay" units="s" minValue="-1" increment="1">Delay (-1 to enable time-of-day fields)</param>
		<param index="2" label="Hour" minValue="-1" maxValue="23" increment="1">hour (24h format, UTC, -1 to ignore)</param>
		<param index="3" label="Minute" minValue="-1" maxValue="59" increment="1">minute (24h format, UTC, -1 to ignore)</param>
		<param index="4" label="Second" minValue="-1" maxValue="59" increment="1">second (24h format, UTC)</param>
		<param index="5">Empty</param>
		<param index="6">Empty</param>
		<param index="7">Empty</param>
*/
static int16_t NavCmd93_DELAY( double freq, uint8_t frame, double params[], NavCmdInf* inf )
{
	if( get_Position_MSStatus() != MS_Ready )
	{	//无定位出错
		inf->counter1 = inf->counter2 = 0;
		return -3;
	}
	
	Position_Control_set_XYLock();
	Position_Control_set_ZLock();
	if( params[0] >= 0 )
	{	//延时指定时间
		if( ++(inf->counter2) >= freq*params[0] )
		{
			inf->counter1 = 0;
			inf->counter2 = 0;
			return -1;
		}
	}
	else
	{	//延时至指定时钟时间
				
		if( params[1]>23 || params[2]>59 || params[3]>59 || params[3]<0 )
		{	//时间信息错误
			inf->counter1 = 0;
			inf->counter2 = 0;
			return -3;
		}
		
		RTC_TimeStruct current_time = Get_RTC_Time();
		if( params[1] >= 0 )
		{
			if( params[2] >= 0 )
			{	//对比时分秒
				uint32_t current_seconds = current_time.Hours*3600 + current_time.Minutes*60 + current_time.Seconds;
				uint32_t target_secongds = params[1]*3600 + params[2]*60 + params[3];
				if( current_seconds >=  target_secongds )
				{
					inf->counter1 = 0;
					inf->counter2 = 0;
					return -1;
				}
			}
			else
			{	//错误
				inf->counter1 = 0;
				inf->counter2 = 0;
				return -3;
			}
		}
		else
		{
			if( params[2] >= 0 )
			{	//对比分秒
				uint32_t current_seconds = current_time.Minutes*60 + current_time.Seconds;
				uint32_t target_secongds = params[2]*60 + params[3];
				if( current_seconds >=  target_secongds )
				{
					inf->counter1 = 0;
					inf->counter2 = 0;
					return -1;
				}
			}
			else
			{	//对比秒
				uint32_t current_seconds = current_time.Seconds;
				uint32_t target_secongds = params[3];
				if( current_seconds >=  target_secongds )
				{
					inf->counter1 = 0;
					inf->counter2 = 0;
					return -1;
				}				
			}
		}
	}
	return -2;
}

static int16_t (*const NavCmdProcess[])( double freq, uint8_t frame, double params[], NavCmdInf* inf ) = 
{
	/*000-*/	0	,
	/*001-*/	0	,
	/*002-*/	0	,
	/*003-*/	0	,
	/*004-*/	0	,
	/*005-*/	0	,
	/*006-*/	0	,
	/*007-*/	0	,
	/*008-*/	0	,
	/*009-*/	0	,
	/*010-*/	0	,
	/*011-*/	0	,
	/*012-*/	0	,
	/*013-*/	0	,
	/*014-*/	0	,
	/*015-*/	0	,
	/*016-*/	NavCmd16_WAYPOINT	,
	/*017-*/	0	,
	/*018-*/	0	,
	/*019-*/	0	,
	/*020-*/	NavCmd20_RETURN_TO_LAUNCH	,
	/*021-*/	0	,
	/*022-*/	NavCmd22_TAKEOFF	,
	/*023-*/	0	,
	/*024-*/	0	,
	/*025-*/	0	,
	/*026-*/	0	,
	/*027-*/	0	,
	/*028-*/	0	,
	/*029-*/	0	,
	/*030-*/	0	,
	/*031-*/	0	,
	/*032-*/	0	,
	/*033-*/	0	,
	/*034-*/	0	,
	/*035-*/	0	,
	/*036-*/	0	,
	/*037-*/	0	,
	/*038-*/	0	,
	/*039-*/	0	,
	/*040-*/	0	,
	/*041-*/	0	,
	/*042-*/	0	,
	/*043-*/	0	,
	/*044-*/	0	,
	/*045-*/	0	,
	/*046-*/	0	,
	/*047-*/	0	,
	/*048-*/	0	,
	/*049-*/	0	,
	/*050-*/	0	,
	/*051-*/	0	,
	/*052-*/	0	,
	/*053-*/	0	,
	/*054-*/	0	,
	/*055-*/	0	,
	/*056-*/	0	,
	/*057-*/	0	,
	/*058-*/	0	,
	/*059-*/	0	,
	/*060-*/	0	,
	/*061-*/	0	,
	/*062-*/	0	,
	/*063-*/	0	,
	/*064-*/	0	,
	/*065-*/	0	,
	/*066-*/	0	,
	/*067-*/	0	,
	/*068-*/	0	,
	/*069-*/	0	,
	/*070-*/	0	,
	/*071-*/	0	,
	/*072-*/	0	,
	/*073-*/	0	,
	/*074-*/	0	,
	/*075-*/	0	,
	/*076-*/	0	,
	/*077-*/	0	,
	/*078-*/	0	,
	/*079-*/	0	,
	/*080-*/	0	,
	/*081-*/	0	,
	/*082-*/	0	,
	/*083-*/	0	,
	/*084-*/	0	,
	/*085-*/	0	,
	/*086-*/	0	,
	/*087-*/	0	,
	/*088-*/	0	,
	/*089-*/	0	,
	/*090-*/	0	,
	/*091-*/	0	,
	/*092-*/	0	,
	/*093-*/	NavCmd93_DELAY	,
	/*094-*/	0	,
	/*095-*/	0	,
	/*096-*/	0	,
	/*097-*/	0	,
	/*098-*/	0	,
	/*099-*/	0	,
	/*100-*/	0	,
	/*101-*/	0	,
	/*102-*/	0	,
	/*103-*/	0	,
	/*104-*/	0	,
	/*105-*/	0	,
	/*106-*/	0	,
	/*107-*/	0	,
	/*108-*/	0	,
	/*109-*/	0	,
	/*110-*/	0	,
	/*111-*/	0	,
	/*112-*/	0	,
	/*113-*/	0	,
	/*114-*/	0	,
	/*115-*/	0	,
	/*116-*/	0	,
	/*117-*/	0	,
	/*118-*/	0	,
	/*119-*/	0	,
	/*120-*/	0	,
	/*121-*/	0	,
	/*122-*/	0	,
	/*123-*/	0	,
	/*124-*/	0	,
	/*125-*/	0	,
	/*126-*/	0	,
	/*127-*/	0	,
	/*128-*/	0	,
	/*129-*/	0	,
	/*130-*/	0	,
	/*131-*/	0	,
	/*132-*/	0	,
	/*133-*/	0	,
	/*134-*/	0	,
	/*135-*/	0	,
	/*136-*/	0	,
	/*137-*/	0	,
	/*138-*/	0	,
	/*139-*/	0	,
	/*140-*/	0	,
	/*141-*/	0	,
	/*142-*/	0	,
	/*143-*/	0	,
	/*144-*/	0	,
	/*145-*/	0	,
	/*146-*/	0	,
	/*147-*/	0	,
	/*148-*/	0	,
	/*149-*/	0	,
	/*150-*/	0	,
	/*151-*/	0	,
	/*152-*/	0	,
	/*153-*/	0	,
	/*154-*/	0	,
	/*155-*/	0	,
	/*156-*/	0	,
	/*157-*/	0	,
	/*158-*/	0	,
	/*159-*/	0	,
	/*160-*/	0	,
	/*161-*/	0	,
	/*162-*/	0	,
	/*163-*/	0	,
	/*164-*/	0	,
	/*165-*/	0	,
	/*166-*/	0	,
	/*167-*/	0	,
	/*168-*/	0	,
	/*169-*/	0	,
	/*170-*/	0	,
	/*171-*/	0	,
	/*172-*/	0	,
	/*173-*/	0	,
	/*174-*/	0	,
	/*175-*/	0	,
	/*176-*/	0	,
	/*177-*/	0	,
	/*178-*/	0	,
	/*179-*/	0	,
	/*180-*/	0	,
	/*181-*/	0	,
	/*182-*/	0	,
	/*183-*/	0	,
	/*184-*/	0	,
	/*185-*/	0	,
	/*186-*/	0	,
	/*187-*/	0	,
	/*188-*/	0	,
	/*189-*/	0	,
	/*190-*/	0	,
	/*191-*/	0	,
	/*192-*/	0	,
	/*193-*/	0	,
	/*194-*/	0	,
	/*195-*/	0	,
	/*196-*/	0	,
	/*197-*/	0	,
	/*198-*/	0	,
	/*199-*/	0	,
	/*200-*/	0	,
	/*201-*/	0	,
	/*202-*/	0	,
	/*203-*/	0	,
	/*204-*/	0	,
	/*205-*/	0	,
	/*206-*/	0	,
	/*207-*/	0	,
	/*208-*/	0	,
	/*209-*/	0	,
	/*210-*/	0	,
	/*211-*/	0	,
	/*212-*/	0	,
	/*213-*/	0	,
	/*214-*/	0	,
	/*215-*/	0	,
	/*216-*/	0	,
	/*217-*/	0	,
	/*218-*/	0	,
	/*219-*/	0	,
	/*220-*/	0	,
	/*221-*/	0	,
	/*222-*/	0	,
	/*223-*/	0	,
	/*224-*/	0	,
	/*225-*/	0	,
	/*226-*/	0	,
	/*227-*/	0	,
	/*228-*/	0	,
	/*229-*/	0	,
	/*230-*/	0	,
	/*231-*/	0	,
	/*232-*/	0	,
	/*233-*/	0	,
	/*234-*/	0	,
	/*235-*/	0	,
	/*236-*/	0	,
	/*237-*/	0	,
	/*238-*/	0	,
	/*239-*/	0	,
	/*240-*/	0	,
	/*241-*/	0	,
	/*242-*/	0	,
	/*243-*/	0	,
	/*244-*/	0	,
	/*245-*/	0	,
	/*246-*/	0	,
	/*247-*/	0	,
	/*248-*/	0	,
	/*249-*/	0	,
	/*250-*/	0	,
	/*251-*/	0	,
	/*252-*/	0	,
	/*253-*/	0	,
	/*254-*/	0	,
	/*255-*/	0	,
	/*256-*/	0	,
	/*257-*/	0	,
	/*258-*/	0	,
	/*259-*/	0	,
	/*260-*/	0	,
	/*261-*/	0	,
	/*262-*/	0	,
	/*263-*/	0	,
	/*264-*/	0	,
	/*265-*/	0	,
	/*266-*/	0	,
	/*267-*/	0	,
	/*268-*/	0	,
	/*269-*/	0	,
	/*270-*/	0	,
	/*271-*/	0	,
	/*272-*/	0	,
	/*273-*/	0	,
	/*274-*/	0	,
	/*275-*/	0	,
	/*276-*/	0	,
	/*277-*/	0	,
	/*278-*/	0	,
	/*279-*/	0	,
	/*280-*/	0	,
	/*281-*/	0	,
	/*282-*/	0	,
	/*283-*/	0	,
	/*284-*/	0	,
	/*285-*/	0	,
	/*286-*/	0	,
	/*287-*/	0	,
	/*288-*/	0	,
	/*289-*/	0	,
	/*290-*/	0	,
	/*291-*/	0	,
	/*292-*/	0	,
	/*293-*/	0	,
	/*294-*/	0	,
	/*295-*/	0	,
	/*296-*/	0	,
	/*297-*/	0	,
	/*298-*/	0	,
	/*299-*/	0	,
	/*300-*/	0	,
};
const uint16_t NavCmdProcess_Count = sizeof( NavCmdProcess ) / sizeof( void* );

int16_t Process_NavCmd( uint16_t cmd, double freq, uint8_t frame, double params[], NavCmdInf* inf )
{
	//无此指令返回错误
	if( cmd>=NavCmdProcess_Count || NavCmdProcess[cmd]==0 )
		return -3;
	
	return NavCmdProcess[cmd]( freq, frame, params, inf );
}