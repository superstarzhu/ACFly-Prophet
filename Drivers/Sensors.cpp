#include "Sensors.hpp"
#include "SensorsBackend.hpp"
#include "Basic.hpp"
#include "Parameters.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "StorageSystem.hpp"
#include "ControlSystem.hpp"

/*IMU*/

	const uint8_t External_Magnetometer_Index = 0;
	const uint8_t Internal_Magnetometer_Index = 2;

	static SemaphoreHandle_t AccelerometersMutex = xSemaphoreCreateMutex();
	static SemaphoreHandle_t GyroscopesMutex = xSemaphoreCreateMutex();
	static SemaphoreHandle_t MagnetometersMutex = xSemaphoreCreateMutex();
	static IMU_Sensor* Accelerometers[IMU_Sensors_Count] = {0};
	static IMU_Sensor* Gyroscopes[IMU_Sensors_Count] = {0};
	static IMU_Sensor* Magnetometers[IMU_Sensors_Count] = {0};
	
	/*IMU传感器注册函数*/
		bool IMUAccelerometerRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] != 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				Accelerometers[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Accelerometers[index];
				sensor->name = name;
				sensor->last_update_time = TIME::now();
				sensor->sensitivity = sensitivity;
				sensor->data.zero();
				sensor->data_raw.zero();
				
				//注册参数
				MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
				IMUConfig initial_cfg;
				initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
				initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
				initial_cfg.STTemperature = 0;
				initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
				ParamGroupRegister( name+"_Acc", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		bool IMUAccelerometerUnRegister( uint8_t index, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				delete Accelerometers[index];
				Accelerometers[index] = 0;
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		
		bool IMUGyroscopeRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] != 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				Gyroscopes[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Gyroscopes[index];
				sensor->name = name;
				sensor->last_update_time = TIME::now();
				sensor->sensitivity = sensitivity;
				sensor->data.zero();
				sensor->data_raw.zero();
				
				//注册参数
				MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
				IMUConfig initial_cfg;
				initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
				initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
				initial_cfg.STTemperature = 0;
				initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
				ParamGroupRegister( name+"_Gyro", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		bool IMUGyroscopeUnRegister( uint8_t index, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				delete Gyroscopes[index];
				Gyroscopes[index] = 0;
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		
		bool IMUMagnetometerRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] != 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				Magnetometers[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Magnetometers[index];
				sensor->name = name;
				sensor->last_update_time = TIME::now();
				sensor->sensitivity = sensitivity;
				sensor->data.zero();
				sensor->data_raw.zero();
				
				//注册参数
				MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
				IMUConfig initial_cfg;
				initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
				initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
				initial_cfg.STTemperature = 0;
				initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
				ParamGroupRegister( name+"_Mag", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
		bool IMUMagnetometerUnRegister( uint8_t index, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				delete Magnetometers[index];
				Magnetometers[index] = 0;
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
	/*IMU传感器注册函数*/
		
	/*IMU传感器更新函数*/
		bool IMUAccelerometerUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
					
			vector3<int32_t> offset(0,0,0);			
			vector3<double> scale(1,1,1);			
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Accelerometers[index];
				//读取参数				
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Acc", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = false;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				sensor->data_rawTC.set_vector( data.x, data.y, data.z );
				sensor->data.x = (sensor->data_raw.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_raw.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_raw.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		bool IMUAccelerometerUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
					
			vector3<int32_t> offset(0,0,0);			
			vector3<double> scale(1,1,1);			
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Accelerometers[index];
				//读取参数				
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Acc", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = true;
				sensor->data_error = data_error;
				sensor->temperature = temperature;
				sensor->data_raw = data;
				double temperature_err = temperature - cfg.STTemperature;
				sensor->data_rawTC.set_vector(
					sensor->data_raw.x - temperature_err*cfg.TemperatureCoefficient[0],
					sensor->data_raw.y - temperature_err*cfg.TemperatureCoefficient[1],
					sensor->data_raw.z - temperature_err*cfg.TemperatureCoefficient[2] );
				sensor->data.x = (sensor->data_rawTC.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_rawTC.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_rawTC.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		
		bool IMUGyroscopeUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Gyroscopes[index];
				//读取参数
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Gyro", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = false;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				sensor->data_rawTC.set_vector( data.x, data.y, data.z );
				sensor->data.x = (sensor->data_raw.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_raw.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_raw.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		bool IMUGyroscopeUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Gyroscopes[index];
				//读取参数
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Gyro", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = true;
				sensor->temperature = temperature;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				double temperature_err = temperature - cfg.STTemperature;
				sensor->data_rawTC.set_vector(
					sensor->data_raw.x - temperature_err*cfg.TemperatureCoefficient[0],
					sensor->data_raw.y - temperature_err*cfg.TemperatureCoefficient[1],
					sensor->data_raw.z - temperature_err*cfg.TemperatureCoefficient[2] );
				sensor->data.x = (sensor->data_rawTC.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_rawTC.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_rawTC.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		
		bool IMUMagnetometerUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Magnetometers[index];
				//读取参数
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Mag", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = false;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				sensor->data_rawTC.set_vector( data.x, data.y, data.z );
				sensor->data.x = (sensor->data_raw.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_raw.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_raw.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
		bool IMUMagnetometerUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Magnetometers[index];
				//读取参数
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Mag", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = true;
				sensor->temperature = temperature;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				double temperature_err = temperature - cfg.STTemperature;
				sensor->data_rawTC.set_vector(
					sensor->data_raw.x - temperature_err*cfg.TemperatureCoefficient[0],
					sensor->data_raw.y - temperature_err*cfg.TemperatureCoefficient[1],
					sensor->data_raw.z - temperature_err*cfg.TemperatureCoefficient[2] );
				sensor->data.x = (sensor->data_rawTC.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_rawTC.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_rawTC.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(MagnetometersMutex);				
				return true;
			}
			return false;
		}
	/*IMU传感器更新函数*/
		
	/*IMU传感器读取函数*/
		bool GetAccelerometer( uint8_t index, IMU_Sensor* sensor, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}				
				*sensor = *Accelerometers[index];
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		bool GetGyroscope( uint8_t index, IMU_Sensor* sensor, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}				
				*sensor = *Gyroscopes[index];
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		bool GetMagnetometer( uint8_t index, IMU_Sensor* sensor, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}				
				*sensor = *Magnetometers[index];
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
	/*IMU传感器读取函数*/
		
/*IMU*/
		
/*位置传感器*/
		
	const uint8_t default_ultrasonic_sensor_index = 1;
	const uint8_t default_optical_flow_index = 8;
	const uint8_t internal_baro_sensor_index = 3;
	const uint8_t default_gps_sensor_index = 6;
		
	static SemaphoreHandle_t Position_Sensors_Mutex[Position_Sensors_Count] = {0};
	static Position_Sensor* Position_Sensors[Position_Sensors_Count];
		
	/*位置传感器注册函数*/
		bool PositionSensorRegister( 
			uint8_t index ,\
			Position_Sensor_Type sensor_type ,\
			Position_Sensor_DataType sensor_data_type ,\
			Position_Sensor_frame sensor_vel_frame ,\
			double delay ,\
			double xy_trustD , \
			double z_trustD , \
			double TIMEOUT \
		)
		{
			if( index >= Position_Sensors_Count )
				return false;			
							
			if( delay < 0 )
				return false;
			if( xy_trustD < 0 )
				return false;
			if( z_trustD < 0 )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{
				if( Position_Sensors[index] != 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				Position_Sensors[index] = new Position_Sensor;
				Position_Sensor* sensor = Position_Sensors[index];
				sensor->available = false;
				sensor->available_status_update_time = TIME::now();
				sensor->last_update_time = TIME::now();
				sensor->sensor_type = sensor_type;
				sensor->sensor_DataType = sensor_data_type;
				sensor->velocity_data_frame = sensor_vel_frame;
				sensor->delay = delay;
				sensor->xy_trustD = xy_trustD;
				sensor->z_trustD = z_trustD;
				sensor->sample_time = -1;
				sensor->mp.initialized = false;
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUnRegister( uint8_t index, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				delete sensor;
				Position_Sensors[index] = 0;
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
	/*位置传感器注册函数*/
	
	//更改位置传感器DataType
	bool PositionSensorChangeDataType( uint8_t index, Position_Sensor_DataType datatype, double TIMEOUT )
	{
		if( index >= Position_Sensors_Count )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
		{	//锁定传感器
			if( Position_Sensors[index] == 0 )
			{
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return false;
			}
			
			Position_Sensor* sensor = Position_Sensors[index];
			sensor->sensor_DataType = datatype;
			
			xSemaphoreGive(Position_Sensors_Mutex[index]);
			return true;
		}	//解锁传感器
		return false;
	}
		
	/*位置传感器更新函数*/
		
		//失能位置传感器
		bool PositionSensorSetInavailable( uint8_t index, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				if( sensor->available != false )
					sensor->available_status_update_time = TIME::now();
				//更新可用状态
				if( sensor->available != false )
					sensor->available_status_update_time = TIME::now();
				sensor->available = false;
				//更新采样时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();			
				
				//记录位置数据
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);				
				return true;
			}	//解锁传感器
			return false;
		}
	
		bool PositionSensorUpdatePositionGlobal( uint8_t index, vector3<double> position_Global, bool available, double delay, double xy_trustD, double z_trustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				
				//判断传感器类型、数据是否正确
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_s_xy:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_z:
						if( __ARM_isnan( position_Global.z ) || __ARM_isinf( position_Global.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_xyz:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || __ARM_isnan( position_Global.z ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || __ARM_isinf( position_Global.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}								
				
				//更新可用状态
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//更新数据
				sensor->position_Global = position_Global;
				//经纬坐标平面投影
				if( available )
				{					
					if( sensor->mp.initialized == false )
					{
						map_projection_init( &sensor->mp , position_Global.x , position_Global.y );
					}
					double pos_x , pos_y;
					map_projection_project( &sensor->mp , position_Global.x , position_Global.y , &pos_x , &pos_y );
					sensor->position.x = pos_x;
					sensor->position.y = pos_y;
					sensor->position.z = position_Global.z;
				}
					
				//更新采样时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();				
				//更新延时时间
				if( delay > 0 )
					sensor->delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//记录位置数据
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUpdatePosition( uint8_t index, vector3<double> position, bool available, double delay, double xy_trustD, double z_trustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				//判断传感器类型、数据是否正确
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_s_xy:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_z:
						if( __ARM_isnan( position.z ) || __ARM_isinf( position.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_xyz:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || __ARM_isnan( position.z ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) || __ARM_isinf( position.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//更新可用状态
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//更新数据
				sensor->position = position;
				
				//更新采样时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();			
				
				//延时大于0更新延时
				if( delay > 0 )
					sensor->delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//记录位置数据
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUpdatePositionGlobalVel( uint8_t index, vector3<double> position_Global, vector3<double> vel, bool available, double delay, double xy_trustD, double z_trustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				//判断传感器类型、数据是否正确
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_sv_xy:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_z:
						if( __ARM_isnan( position_Global.z ) || __ARM_isinf( position_Global.z ) || \
								__ARM_isnan( vel.z ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_xyz:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || __ARM_isnan( position_Global.z ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || __ARM_isinf( position_Global.z ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || __ARM_isnan( vel.z ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//更新可用状态
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//更新数据
				sensor->position_Global = position_Global;
				sensor->velocity = vel;
				//经纬坐标平面投影				
				if( available )
				{
					if( sensor->mp.initialized == false )
					{
						map_projection_init( &sensor->mp , position_Global.x , position_Global.y );
					}
					double pos_x , pos_y;
					map_projection_project( &sensor->mp , position_Global.x , position_Global.y , &pos_x , &pos_y );
					sensor->position.x = pos_x;
					sensor->position.y = pos_y;
					sensor->position.z = position_Global.z;
				}
					
				//更新采样时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();				
				//更新延时时间
				if( delay > 0 )
					sensor->delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//记录位置数据
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUpdatePositionVel( uint8_t index, vector3<double> position, vector3<double> vel, bool available, double delay, double xy_trustD, double z_trustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_sv_xy:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_z:
						if( __ARM_isnan( position.z ) || __ARM_isinf( position.z ) || \
								__ARM_isnan( vel.z ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_xyz:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || __ARM_isnan( position.z ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) || __ARM_isinf( position.z ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || __ARM_isnan( vel.z ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//更新可用状态
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//更新数据
				sensor->position = position;
				sensor->velocity = vel;
				
				//更新采样时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();			
				
				//延时大于0更新延时
				if( delay > 0 )
					sensor->delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//记录位置数据
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUpdateVel( uint8_t index, vector3<double> vel, bool available, double delay, double xy_trustD, double z_trustD, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				
				//判断传感器类型、数据是否正确
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_v_xy:
						if( __ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_v_z:
						if( __ARM_isnan( vel.z ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_v_xyz:
						if( __ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || __ARM_isnan( vel.z ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//更新可用状态
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//更新数据
				sensor->velocity = vel;
				
				//更新采样时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();			
				
				//延时大于0更新延时
				if( delay > 0 )
					sensor->delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//记录位置数据
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
	/*位置传感器更新函数*/
		
	/*位置传感器读取函数*/
		bool GetPositionSensor( uint8_t index, Position_Sensor* result_sensor, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				*result_sensor = *sensor;
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
	/*位置传感器读取函数*/
	
/*位置传感器*/
		
void init_Sensors()
{
	for( int i = 0; i < Position_Sensors_Count; ++i )
		Position_Sensors_Mutex[i] = xSemaphoreCreateMutex();
}