#pragma once

#include "Basic.hpp"
#include "mavlink.h"

//Nav命令信息
struct NavCmdInf
{
	uint32_t counter1;
	uint32_t counter2;
};
//首次执行NavCmd命令前需要初始化NavCmdInf
inline void init_NavCmdInf( NavCmdInf* inf )
{
	inf->counter1 = inf->counter2 = 0;
}

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
int16_t Process_NavCmd( uint16_t cmd, double freq, uint8_t frame, double params[], NavCmdInf* inf );

/*frame定义：
	<entry value="0" name="MAV_FRAME_GLOBAL">
		<description>Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL).</description>
	</entry>
	<entry value="1" name="MAV_FRAME_LOCAL_NED">
		<description>Local coordinate frame, Z-down (x: north, y: east, z: down).</description>
	</entry>
	<entry value="2" name="MAV_FRAME_MISSION">
		<description>NOT a coordinate frame, indicates a mission command.</description>
	</entry>
	<entry value="3" name="MAV_FRAME_GLOBAL_RELATIVE_ALT">
		<description>Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.</description>
	</entry>
	<entry value="4" name="MAV_FRAME_LOCAL_ENU">
		<description>Local coordinate frame, Z-up (x: east, y: north, z: up).</description>
	</entry>
	<entry value="5" name="MAV_FRAME_GLOBAL_INT">
		<description>Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL).</description>
	</entry>
	<entry value="6" name="MAV_FRAME_GLOBAL_RELATIVE_ALT_INT">
		<description>Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.</description>
	</entry>
	<entry value="7" name="MAV_FRAME_LOCAL_OFFSET_NED">
		<description>Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.</description>
	</entry>
	<entry value="8" name="MAV_FRAME_BODY_NED">
		<deprecated since="2019-08" replaced_by="MAV_FRAME_BODY_FRD"/>
		<description>Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.</description>
	</entry>
	<entry value="9" name="MAV_FRAME_BODY_OFFSET_NED">
		<deprecated since="2019-08" replaced_by="MAV_FRAME_BODY_FRD"/>
		<description>Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.</description>
	</entry>
	<entry value="10" name="MAV_FRAME_GLOBAL_TERRAIN_ALT">
		<description>Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.</description>
	</entry>
	<entry value="11" name="MAV_FRAME_GLOBAL_TERRAIN_ALT_INT">
		<description>Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.</description>
	</entry>
	<entry value="12" name="MAV_FRAME_BODY_FRD">
		<description>Body fixed frame of reference, Z-down (x: forward, y: right, z: down).</description>
	</entry>
	<entry value="13" name="MAV_FRAME_BODY_FLU">
		<description>Body fixed frame of reference, Z-up (x: forward, y: left, z: up).</description>
	</entry>
	<entry value="14" name="MAV_FRAME_MOCAP_NED">
		<deprecated since="2019-08" replaced_by="MAV_FRAME_LOCAL_FRD"/>
		<description>Odometry local coordinate frame of data given by a motion capture system, Z-down (x: north, y: east, z: down).</description>
	</entry>
	<entry value="15" name="MAV_FRAME_MOCAP_ENU">
		<deprecated since="2019-08" replaced_by="MAV_FRAME_LOCAL_FLU"/>
		<description>Odometry local coordinate frame of data given by a motion capture system, Z-up (x: east, y: north, z: up).</description>
	</entry>
	<entry value="16" name="MAV_FRAME_VISION_NED">
		<deprecated since="2019-08" replaced_by="MAV_FRAME_LOCAL_FRD"/>
		<description>Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: north, y: east, z: down).</description>
	</entry>
	<entry value="17" name="MAV_FRAME_VISION_ENU">
		<deprecated since="2019-08" replaced_by="MAV_FRAME_LOCAL_FLU"/>
		<description>Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: east, y: north, z: up).</description>
	</entry>
	<entry value="18" name="MAV_FRAME_ESTIM_NED">
		<deprecated since="2019-08" replaced_by="MAV_FRAME_LOCAL_FRD"/>
		<description>Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: north, y: east, z: down).</description>
	</entry>
	<entry value="19" name="MAV_FRAME_ESTIM_ENU">
		<deprecated since="2019-08" replaced_by="MAV_FRAME_LOCAL_FLU"/>
		<description>Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: east, y: noth, z: up).</description>
	</entry>
	<entry value="20" name="MAV_FRAME_LOCAL_FRD">
		<description>Forward, Right, Down coordinate frame. This is a local frame with Z-down and arbitrary F/R alignment (i.e. not aligned with NED/earth frame).</description>
	</entry>
	<entry value="21" name="MAV_FRAME_LOCAL_FLU">
		<description>Forward, Left, Up coordinate frame. This is a local frame with Z-up and arbitrary F/L alignment (i.e. not aligned with ENU/earth frame).</description>
	</entry>
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

/*NavCmd93_DELAY
	MAV_CMD_NAV_DELAY
	延时
	参数:
		<description>Delay the next navigation command a number of seconds or until a specified time</description>
		<param index="0" label="Delay" units="s" minValue="-1" increment="1">Delay (-1 to enable time-of-day fields)</param>
		<param index="1" label="Hour" minValue="-1" maxValue="23" increment="1">hour (24h format, UTC, -1 to ignore)</param>
		<param index="2" label="Minute" minValue="-1" maxValue="59" increment="1">minute (24h format, UTC, -1 to ignore)</param>
		<param index="3" label="Second" minValue="-1" maxValue="59" increment="1">second (24h format, UTC)</param>
		<param index="4">Empty</param>
		<param index="5">Empty</param>
		<param index="6">Empty</param>
*/
