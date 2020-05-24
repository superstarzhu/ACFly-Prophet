#pragma once

#include "Basic.hpp"
#include "vector3.hpp"
#include "map_projection.hpp"

/*IMU*/
	struct IMUConfig
	{
		double offset[3];
		double scale[3];
		double STTemperature;
		double TemperatureCoefficient[3];
	} __PACKED;
	#define IMUConfigLength 10

	/*IMU传感器定义*/
		#define IMU_Sensors_Count 3
		
		extern const uint8_t External_Magnetometer_Index;
		extern const uint8_t Internal_Magnetometer_Index;
		struct IMU_Sensor
		{
			SName name;	//传感器名称
			TIME last_update_time;	//上次更新时间
			double sample_time;	//采样时间
			bool calibrated;	//是否已校准
			bool have_temperature;	//是否有温度数据
			
			double temperature;	//温度
			double sensitivity;	//灵敏度（原始数据->实际单位 陀螺：rad/s 加速度：cm/s^2 磁场：gauss）
			
			bool data_error;	//数据是否错误（爆量程）
			vector3<int32_t> data_raw;	//原始数据
			vector3<double> data_rawTC;	//温度补偿后的原始数据
			vector3<double> data;	//实际单位数据
		};		
	/*IMU传感器定义*/
		
	/*IMU传感器读取函数*/
		bool GetAccelerometer( uint8_t index, IMU_Sensor* sensor, double TIMEOUT = -1 );
		bool GetGyroscope( uint8_t index, IMU_Sensor* sensor, double TIMEOUT = -1 );
		bool GetMagnetometer( uint8_t index, IMU_Sensor* sensor, double TIMEOUT = -1 );
	/*IMU传感器读取函数*/
/*IMU*/
	
/*位置传感器*/
	/*位置传感器定义*/
		#define Position_Sensors_Count 16
		
		extern const uint8_t default_ultrasonic_sensor_index;
		extern const uint8_t default_optical_flow_index;
		extern const uint8_t internal_baro_sensor_index;
		extern const uint8_t default_gps_sensor_index;
		
		enum Position_Sensor_Type
		{
			Position_Sensor_Type_GlobalPositioning = 0 ,	//全球定位（经纬度定位，如GPS）
			Position_Sensor_Type_RelativePositioning = 1 ,	//相对定位（如气压计，参照物不会改变）
			Position_Sensor_Type_RangePositioning = 2 ,	//距离定位（测距定位，如超声波，参照物可能会变化）
		};
		enum Position_Sensor_DataType
		{
			//s-位置 v-速度
			//如sv_xy表示该传感器具有：位置速度的xy数据
			Position_Sensor_DataType_s_xy = 0 ,
			Position_Sensor_DataType_s_z = 1 ,
			Position_Sensor_DataType_s_xyz = 2  ,
			
			Position_Sensor_DataType_v_xy = 8  ,
			Position_Sensor_DataType_v_z = 9  ,
			Position_Sensor_DataType_v_xyz = 10 ,
			
			Position_Sensor_DataType_sv_xy = 16  ,
			Position_Sensor_DataType_sv_z = 17  ,
			Position_Sensor_DataType_sv_xyz = 18  ,
		};
		enum Position_Sensor_frame
		{
			Position_Sensor_frame_ENU = 0 ,	//速度数据在ENU坐标系下
			Position_Sensor_frame_BodyHeading = 1 ,	//速度数据x为机头朝向（与地面平行），y为朝向机头左方（与地面平行），z为上方
		};
		struct Position_Sensor
		{
			bool available;	//传感器是否可用
			TIME last_update_time;	//上次更新时间
			TIME available_status_update_time;	//传感器可用信号更新时间
			double delay;	//传感器延时
			double xy_trustD;	//信任度 0最高信任度
			double z_trustD;	//信任度 0最高信任度
			double sample_time;	//采样时间

			Position_Sensor_Type sensor_type;	//传感器类型（见枚举注释）
			Position_Sensor_DataType sensor_DataType;	//传感器数据类型（见枚举注释）
			Position_Sensor_frame velocity_data_frame;	//速度数据坐标系（见枚举注释）
			
			vector3<double> position_Global;	//经纬度
			vector3<double> position;	//位置(cm)
			vector3<double> velocity;	//速度(cm/s)
			
			Map_Projection mp;
		};
	/*位置传感器定义*/
	
	/*位置传感器读取函数*/
		bool GetPositionSensor( uint8_t index, Position_Sensor* result_sensor, double TIMEOUT = -1 );
	/*位置传感器读取函数*/
/*位置传感器*/