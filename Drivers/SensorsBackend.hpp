#pragma once

#include "Sensors.hpp"

void init_Sensors();

/*IMU*/
	/*IMU传感器注册函数*/
		bool IMUAccelerometerRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT = -1 );
		bool IMUAccelerometerUnRegister( uint8_t index, double TIMEOUT = -1 );
		
		bool IMUGyroscopeRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT = -1 );
		bool IMUGyroscopeUnRegister( uint8_t index, double TIMEOUT = -1 );
		
		bool IMUMagnetometerRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT = -1 );
		bool IMUMagnetometerUnRegister( uint8_t index, double TIMEOUT = -1 );
	/*IMU传感器注册函数*/
		
	/*IMU传感器更新函数*/
		bool IMUAccelerometerUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT = -1 );
		bool IMUAccelerometerUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT = -1 );
		
		bool IMUGyroscopeUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT = -1 );
		bool IMUGyroscopeUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT = -1 );
			
		bool IMUMagnetometerUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT = -1 );
		bool IMUMagnetometerUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT = -1 );
	/*IMU传感器更新函数*/
/*IMU*/

/*位置传感器*/
	/*位置传感器注册函数*/
		//参数详细定义见Sensors.h中位置传感器注释
		bool PositionSensorRegister( 
			uint8_t index ,\
			Position_Sensor_Type sensor_type ,\
			Position_Sensor_DataType sensor_data_type ,\
			Position_Sensor_frame sensor_vel_frame ,\
			double delay ,\
			double xy_trustD = 0 ,\
			double z_trustD = 0 , \
			double TIMEOUT = -1 \
		);
		bool PositionSensorUnRegister( uint8_t index, double TIMEOUT = -1 );
	/*位置传感器注册函数*/
	
	//更改位置传感器DataType
	bool PositionSensorChangeDataType( uint8_t index, Position_Sensor_DataType datatype, double TIMEOUT = -1 );
		
	/*位置传感器更新函数*/		
		//失能位置传感器
		bool PositionSensorSetInavailable( uint8_t index, double TIMEOUT = -1 );
	
		//delay参数小于0则不会改变delay
		bool PositionSensorUpdatePositionGlobal( uint8_t index, vector3<double> position_Global, bool available, double delay = -1.0, double xy_trustD = -1, double z_trustD = -1, double TIMEOUT = -1 );
		bool PositionSensorUpdatePosition( uint8_t index, vector3<double> position, bool available, double delay = -1.0, double xy_trustD = -1, double z_trustD = -1, double TIMEOUT = -1 );
		bool PositionSensorUpdatePositionGlobalVel( uint8_t index, vector3<double> position_Global, vector3<double> vel, bool available, double delay = -1.0, double xy_trustD = -1, double z_trustD = -1, double TIMEOUT = -1 );
		bool PositionSensorUpdatePositionVel( uint8_t index, vector3<double> position, vector3<double> vel, bool available, double delay = -1.0, double xy_trustD = -1, double z_trustD = -1, double TIMEOUT = -1 );
		bool PositionSensorUpdateVel( uint8_t index, vector3<double> vel, bool available, double delay = -1.0, double xy_trustD = -1, double z_trustD = -1, double TIMEOUT = -1 );
	/*位置传感器更新函数*/
/*位置传感器*/