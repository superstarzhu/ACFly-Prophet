#include "MavlinkSendFuncs.hpp"
#include "mavlink.h"

#include "Basic.hpp"
#include "AC_Math.hpp"
#include "MeasurementSystem.hpp"
#include "Sensors.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"

static bool Msg01_SYS_STATUS( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Attitude_MSStatus() != MS_Ready )
		return false;
	
	float volt, current, RMPercent;
	get_MainBatteryInf( &volt, &current, 0, 0, &RMPercent );
	mavlink_msg_sys_status_pack_chan(
		1 ,	//system id
		MAV_COMP_ID_AUTOPILOT1 ,	//component id
		port , 	//chan
		msg_sd ,
		MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_MAG ,
		MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_MAG ,
		0 ,
		getCPULoad() * 10 ,
		volt*1000 ,
		current*100 ,
		RMPercent ,
		0 ,
		0 ,
		0 ,
		0 ,
		0 ,
		0
	);
	return true;
}

static bool Msg30_ATTITUDE( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Attitude_MSStatus() != MS_Ready )
		return false;
	
	Quaternion airframe_quat;
	get_Attitude_quat(&airframe_quat);
	airframe_quat.Enu2Ned();
	vector3<double> angular_rate;
	get_AngularRate_Ctrl(&angular_rate);
	
	mavlink_msg_attitude_pack_chan( 
		1 ,	//system id
		MAV_COMP_ID_AUTOPILOT1 ,	//component id
		port , 	//chan
		msg_sd ,
		TIME::get_System_Run_Time() * 1000 , 	//boot ms
		airframe_quat.getRoll() ,
		airframe_quat.getPitch() ,
		airframe_quat.getYaw() ,
		angular_rate.x , 
		angular_rate.y ,
		angular_rate.z );
	
	return true;
}

static bool Msg31_ATTITUDE_QUATERNION( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Attitude_MSStatus() != MS_Ready )
		return false;
	
	Quaternion airframe_quat;
	get_Attitude_quat(&airframe_quat);
	airframe_quat.Enu2Ned();
	vector3<double> angular_rate;
	get_AngularRate_Ctrl(&angular_rate);
	
	mavlink_msg_attitude_quaternion_pack_chan( 
		1 ,	//system id
		MAV_COMP_ID_AUTOPILOT1 ,	//component id
		port , 	//chan
		msg_sd ,
		TIME::get_System_Run_Time() * 1000 , 	//boot ms
		airframe_quat.get_qw() ,
		airframe_quat.get_qx() ,
		airframe_quat.get_qy() ,
		airframe_quat.get_qz() ,
		angular_rate.x , 
		angular_rate.y ,
		angular_rate.z );
	
	return true;
}

static bool Msg24_GPS_RAW_INT( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Altitude_MSStatus() != MS_Ready )
		return false;
		
	PosSensorHealthInf2 posInf;
	if( get_OptimalGlobal_XY(&posInf) )
	{
		int8_t gps_fix = 0;
		double lat = 0, lon = 0, alt = 0;
		
		Position_Sensor gps_sensor;
		if( GetPositionSensor( posInf.sensor_ind, &gps_sensor ) )
		{
			lat = gps_sensor.position_Global.x;
			lon = gps_sensor.position_Global.y;
			switch( gps_sensor.sensor_DataType )
			{
				case Position_Sensor_DataType_sv_xyz:
				case Position_Sensor_DataType_s_xyz:
					alt = gps_sensor.position_Global.z;
					gps_fix = 3;
					break;
				case Position_Sensor_DataType_sv_xy:
				case Position_Sensor_DataType_s_xy:
					gps_fix = 2;
					break; 
				default:
					break;
			}
		}
		
		mavlink_msg_gps_raw_int_pack_chan( 
			1 ,	//system id
			MAV_COMP_ID_AUTOPILOT1 ,	//component id
			port	,	//chan
			msg_sd,
			TIME::get_System_Run_Time() * 1e6 ,	//usec
			gps_fix ,	//fixtype
			lat*1e7  ,	//lat
			lon*1e7  ,	//lon
			alt ,	//alt
			0 ,	//eph
			0 ,	//epv
			0 ,	//vel
			0 ,	//cog
			0 ,	//satellites_visible
			0 ,	//alt_ellipsoid
			0 ,	//h_acc
			0 ,	//v_acc
			0 ,	//vel_acc
			0  //hdg_acc
		);
		return true;
	}			
	return false;
}

static bool Msg33_GLOBAL_POSITION_INT( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Attitude_MSStatus() != MS_Ready )
		return false;
	
	double lat = 0, lon = 0, alt = 0;
	
	PosSensorHealthInf2 global_posInf;
	if( get_OptimalGlobal_XY(&global_posInf) )
	{		
		map_projection_reproject( &global_posInf.mp, 
				global_posInf.PositionENU.x+global_posInf.HOffset.x, 
				global_posInf.PositionENU.y+global_posInf.HOffset.y,
				&lat, &lon );
	}
	
	PosSensorHealthInf1 z_posInf;
	if( get_OptimalGlobal_Z(&z_posInf) )
		alt = z_posInf.PositionENU.z + z_posInf.HOffset;
	
	double homeZ;
	getHomeLocalZ(&homeZ);
	double heightAboveGround = z_posInf.PositionENU.z - homeZ;
	
	vector3<double> vel;
	get_VelocityENU(&vel);
	
	mavlink_msg_global_position_int_pack_chan( 
		1 ,	//system id
		MAV_COMP_ID_AUTOPILOT1 ,	//component id
		port	,	//chan
		msg_sd,
		TIME::get_System_Run_Time() * 1e3 ,	//usec
		lat*1e7  ,	//lat
		lon*1e7  ,	//lon
		alt*10 ,	//alt
		heightAboveGround*10 ,	//Altitude above ground
		vel.y ,	//vel north
		vel.x ,	//vel east
		-vel.z ,	//vel down
		0xffff	//Vehicle heading
	);
	
	return true;
}

static bool Msg32_LOCAL_POSITION_NED( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Altitude_MSStatus() != MS_Ready )
		return false;
	
	vector3<double> Position;
	get_Position(&Position);
	vector3<double> VelocityENU;
	get_VelocityENU(&VelocityENU);
	
	mavlink_msg_local_position_ned_pack_chan( 
		1 ,	//system id
		MAV_COMP_ID_AUTOPILOT1 ,	//component id
		port , 	//chan
		msg_sd ,
		TIME::get_System_Run_Time() * 1000 , 	//boot ms
		Position.y * 0.01f ,
		Position.x * 0.01f ,
		Position.z * -0.01f ,
		VelocityENU.y * 0.01f , 
		VelocityENU.x * 0.01f ,
		VelocityENU.z * -0.01f);
	return true;
}

bool (*const Mavlink_Send_Funcs[])( uint8_t port , mavlink_message_t* msg_sd ) = 
{
	/*000-*/	0	,
	/*001-*/	Msg01_SYS_STATUS	,
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
	/*016-*/	0	,
	/*017-*/	0	,
	/*018-*/	0	,
	/*019-*/	0	,
	/*020-*/	0	,
	/*021-*/	0	,
	/*022-*/	0	,
	/*023-*/	0	,
	/*024-*/	Msg24_GPS_RAW_INT	,
	/*025-*/	0	,
	/*026-*/	0	,
	/*027-*/	0	,
	/*028-*/	0	,
	/*029-*/	0	,
	/*030-*/	Msg30_ATTITUDE	,
	/*031-*/	Msg31_ATTITUDE_QUATERNION	,
	/*032-*/	Msg32_LOCAL_POSITION_NED	,
	/*033-*/	Msg33_GLOBAL_POSITION_INT	,
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
	/*093-*/	0	,
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
};
const uint16_t Mavlink_Send_Funcs_Count = sizeof( Mavlink_Send_Funcs ) / sizeof( void* );