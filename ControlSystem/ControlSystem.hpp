#pragma once

#include "Basic.hpp"
#include "vector2.hpp"
#include "vector3.hpp"

/*观测接口*/
	bool get_hover_throttle( double* result, double TIMEOUT = -1 );
	bool get_is_inFlight( bool* result, double TIMEOUT = -1 );
	bool get_WindDisturbance( vector3<double>* result, double TIMEOUT = -1 );
/*观测接口*/	

/*Home位置*/
	bool getHomeLocalZ( double* home, double TIMEOUT = -1 );
	bool getHomePoint( vector2<double>* home, double TIMEOUT = -1 );
	bool getHomeLatLon( vector2<double>* home, double TIMEOUT = -1 );
/*Home位置*/

/*姿态控制*/
	enum Attitude_ControlMode
	{
		Attitude_ControlMode_Angle ,
		Attitude_ControlMode_AngularRate ,
		Attitude_ControlMode_Locking ,
	};
	
	//获取姿态控制器是否打开
	bool is_Attitude_Control_Enabled( bool* enabled, double TIMEOUT = -1 );
	//打开关闭姿态控制器
	bool Attitude_Control_Enable( double TIMEOUT = -1 );
	bool Attitude_Control_Disable( double TIMEOUT = -1 );

	//获取当前油门
	bool get_Target_Throttle( double* result, double TIMEOUT = -1 );
	//设定油门
	bool Attitude_Control_set_Throttle( double thr, double TIMEOUT = -1 );
	//获取目标Roll Pitch
	bool Attitude_Control_get_Target_RollPitch( double* Roll, double* Pitch, double TIMEOUT = -1 );
	//设定目标Roll Pitch
	bool Attitude_Control_set_Target_RollPitch( double Roll, double Pitch, double TIMEOUT = -1 );

	//设定目标Yaw
	bool Attitude_Control_set_Target_Yaw( double Yaw, double TIMEOUT = -1 );
	bool Attitude_Control_set_Target_YawRelative( double Yaw, double TIMEOUT = -1 );
	//设定目标Yaw速度
	bool Attitude_Control_set_Target_YawRate( double YawRate, double TIMEOUT = -1 );
	//锁定Yaw（刹车后锁角度）
	bool Attitude_Control_set_YawLock( double TIMEOUT = -1 );
/*姿态控制*/

/*位置控制*/
	enum Position_ControlMode
	{
		//控制器未打开
		Position_ControlMode_Null = 255 ,
		
		//普通模式	
		Position_ControlMode_Position = 12 ,	//位置锁定模式
		Position_ControlMode_Velocity = 11 ,	//速度控制模式
		Position_ControlMode_Locking = 10 ,	//刹车后锁位置
		
		//2D自动模式
		Position_ControlMode_Takeoff = 20 ,	//起飞模式
		Position_ControlMode_RouteLine = 22 ,	//巡线模式
		
		//3D自动模式
		Position_ControlMode_RouteLine3D = 52 ,	//巡线模式
	};
	#define Is_2DAutoMode(x) (x>=20 && x<=49)
	#define Is_3DAutoMode(x) (x>=50 && x<=79)
	
	/*高度*/
		//打开关闭高度控制器
		bool is_Altitude_Control_Enabled( bool* ena, double TIMEOUT = -1 );
		bool Altitude_Control_Enable( double TIMEOUT = -1 );
		bool Altitude_Control_Disable( double TIMEOUT = -1 );
	
		//获取当前高度控制模式
		bool get_Altitude_ControlMode( Position_ControlMode* mode, double TIMEOUT = -1 );
	
		//设定Z自动飞行速度
		bool Position_Control_set_ZAutoSpeed( double AtVelZ, double TIMEOUT = -1 );
	
		//移动Z目标位置（仅能在位置或者自动模式中使用）
		bool Position_Control_move_TargetPositionZRelative( double posz, double TIMEOUT = -1 );
	
		//设定目标高度
		bool Position_Control_set_TargetPositionZ( double posz, double TIMEOUT = -1 );
		//设定目标高度(相对当前上升或下降）
		bool Position_Control_set_TargetPositionZRelative( double posz, double TIMEOUT = -1 );
		//设定目标高度（指定海拔高度）
		bool Position_Control_set_TargetPositionZGlobal( double posz, double TIMEOUT = -1 );
		//设定目标高度（相对起飞点）
		bool Position_Control_set_TargetPositionZRA( double posz, double TIMEOUT = -1 );
		
		//设定目标垂直速度
		bool Position_Control_set_TargetVelocityZ( double velz, double TIMEOUT = -1 );
		//刹车后锁高度
		bool Position_Control_set_ZLock( double TIMEOUT = -1 );		
	
		//起飞到当前高度上方的height高度
		bool Position_Control_Takeoff_HeightRelative( double height, double TIMEOUT = -1 );
		//起飞到指定海拔高度
		bool Position_Control_Takeoff_HeightGlobal( double height, double TIMEOUT = -1 );
		//起飞到指定高度（Local坐标）
		bool Position_Control_Takeoff_Height( double height, double TIMEOUT = -1 );
	/*高度*/
	
	/*水平位置*/
		//打开关闭水平位置控制
		bool is_Position_Control_Enabled( bool* ena, double TIMEOUT = -1 );
		bool Position_Control_Enable( double TIMEOUT = -1 );
		bool Position_Control_Disable( double TIMEOUT = -1 );
		
		//获取当前水平位置控制模式
		bool get_Position_ControlMode( Position_ControlMode* mode, double TIMEOUT = -1 );
	
		//设定自动飞行速度
		bool Position_Control_set_XYAutoSpeed( double AtVelXY, double TIMEOUT = -1 );
		bool Position_Control_set_XYZAutoSpeed( double AtVelXYZ, double TIMEOUT = -1 );
	
		//设定目标水平位置
		bool Position_Control_set_TargetPositionXY( double posx, double posy, double TIMEOUT = -1 );
		bool Position_Control_set_TargetPositionXYZ( double posx, double posy, double posz, double TIMEOUT = -1 );
		//设定目标水平位置（相对当前坐标）
		bool Position_Control_set_TargetPositionXYRelative( double posx, double posy, double TIMEOUT = -1 );
		bool Position_Control_set_TargetPositionXYZRelative( double posx, double posy, double posz, double TIMEOUT = -1 );
		//设定目标水平位置（相对当前坐标且偏移在Bodyheading系下）
		bool Position_Control_set_TargetPositionXYRelativeBodyheading( double posx, double posy, double TIMEOUT = -1 );
		bool Position_Control_set_TargetPositionXYZRelativeBodyheading( double posx, double posy, double posz, double TIMEOUT = -1 );
		//根据经纬度（根据最优全球定位传感器）设定目标水平位置
		bool Position_Control_set_TargetPositionXY_LatLon( double Lat, double Lon, double TIMEOUT = -1 );
		//根据经纬度和海拔高度（根据最优全球定位传感器）设定目标位置（三维直线）
		bool Position_Control_set_TargetPositionXYZ_LatLon( double Lat, double Lon, double posz, double TIMEOUT = -1 );
		//根据经纬度（根据最优全球定位传感器）设定目标位置（三维直线）
		//posz为相对当前高度增加的高度
		bool Position_Control_set_TargetPositionXYZRelative_LatLon( double Lat, double Lon, double posz, double TIMEOUT = -1 );
		//根据经纬度（根据最优全球定位传感器）设定目标位置（三维直线）
		//posz为距离起飞点Z坐标的高度
		bool Position_Control_set_TargetPositionXYZRA_LatLon( double Lat, double Lon, double posz, double TIMEOUT = -1 );
		
		//设定目标水平速度（ENU朝向）并限制最大角度（补偿风力后的角度）
		//maxAngle<0不限制角度
		bool Position_Control_set_TargetVelocityXY_AngleLimit( double velx, double vely, double maxAngle = -1, double TIMEOUT = -1 );
		//设定目标水平速度（Bodyheading朝向）并限制最大角度（补偿风力后的角度）
		//maxRoll>0且maxPitch<0时限制合角度
		bool Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( double velx, double vely, double maxRoll = -1, double maxPitch = -1, double TIMEOUT = -1 );
		//刹车后锁定水平位置
		bool Position_Control_set_XYLock( double TIMEOUT = -1 );
	/*水平位置*/
/*位置控制*/

/*安全接口*/
	//获取控制器上次控制时间
	//太久不进行控制将进入MSafe模式
	//XY为水平控制（包括姿态控制）
	//Z为高度控制（包括油门控制）
	bool get_lastXYCtrlTime( TIME* t, double TIMEOUT = -1 );
	bool get_lastZCtrlTime( TIME* t, double TIMEOUT = -1 );
	
	//将控制器上次控制时间设为不可用
	//强制进入MSafe模式
	//MSafe模式下用户无法关闭位置控制器
	//同时作出XYZ位置控制可退出MSafe
	//（水平控制不可用时控制角度）
	bool enter_MSafe( double TIMEOUT = -1 );
	
	//获取是否进入了MSafe模式
	bool is_MSafeCtrl();
/*安全接口*/