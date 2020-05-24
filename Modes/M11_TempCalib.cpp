#include "M11_TempCalib.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "drv_Sensors.hpp"

//温度系数拟合校准
//静止计算温度系数
//！！中途请勿中断否则可能造成硬件烧毁！！

//朱文杰 20181226
//请勿用于商业用途
//！！抄袭必究！！

M11_TempCalib::M11_TempCalib():Mode_Base( "TempCalib", 11 )
{
	
}

ModeResult M11_TempCalib::main_func( void* param1, uint32_t param2 )
{
	setLedMode(LEDMode_Processing2);
	
	//判断板子是否静止
	vector3<double> Calibration_Acc_Max , Calibration_Acc_Min;
	vector3<double> Calibration_Gyro_Max , Calibration_Gyro_Min;
	
	//初始化静止检测
	vector3<double> acc_filted;
	vector3<double> gyro_filted;
	get_AngularRate_Ctrl(&gyro_filted);
	get_AccelerationENU_Ctrl(&acc_filted);
	Calibration_Acc_Max = Calibration_Acc_Min = acc_filted;
	Calibration_Gyro_Max = Calibration_Gyro_Min = gyro_filted;
	
	//最小最大温度点
	double gyro_min_temperature[IMU_Sensors_Count];
	double gyro_max_temperature[IMU_Sensors_Count];
	double acc_min_temperature[IMU_Sensors_Count];
	double acc_max_temperature[IMU_Sensors_Count];
	//校准状态机
	uint8_t calibration_step = 0;
	TIME calibration_start_time = TIME::now();
	//校准值
	uint32_t calib_n = 0;
	bool calib_gyroscope[IMU_Sensors_Count];
	bool calib_accelerometer[IMU_Sensors_Count];
	double sum_gyro_t[IMU_Sensors_Count];
	double sum_gyro_t2[IMU_Sensors_Count];
	double sum_acc_t[IMU_Sensors_Count];
	double sum_acc_t2[IMU_Sensors_Count];
	vector3<double> sum_gyro_y[IMU_Sensors_Count];
	vector3<double> sum_gyro_ty[IMU_Sensors_Count];
	vector3<double> sum_acc_y[IMU_Sensors_Count];
	vector3<double> sum_acc_ty[IMU_Sensors_Count];
	
	//初始化校准
	for( uint8_t i = 0; i < IMU_Sensors_Count; ++i )
	{
		IMU_Sensor sensor;
		//陀螺
		if( GetGyroscope( i, &sensor ) )
		{
			if(sensor.have_temperature)
			{
				calib_gyroscope[i] = true;
				sum_gyro_t[i] = sum_gyro_t2[i] = 0;
				sum_gyro_y[i].zero();
				sum_gyro_ty[i].zero();
				
				gyro_min_temperature[i] = gyro_max_temperature[i] = sensor.temperature;
			}
			else
				calib_gyroscope[i] = false;
		}
		else
			calib_gyroscope[i] = false;
		//加速度计
		if( GetAccelerometer( i, &sensor ) )
		{
			if(sensor.have_temperature)
			{
				calib_accelerometer[i] = true;
				sum_acc_t[i] = sum_acc_t2[i] = 0;
				sum_acc_y[i].zero();
				sum_acc_ty[i].zero();
				
				acc_min_temperature[i] = acc_max_temperature[i] = sensor.temperature;
			}
			else
				calib_accelerometer[i] = false;
		}
		else
			calib_accelerometer[i] = false;
	}
	
	while(1)
	{
		os_delay(0.01);
		
		/*静止检测*/
			vector3<double> acc_filted;
			vector3<double> gyro_filted;
			get_AngularRate_Ctrl(&gyro_filted);
			get_AccelerationENU_Ctrl(&acc_filted);
			
			if( acc_filted.x > Calibration_Acc_Max.x ) Calibration_Acc_Max.x = acc_filted.x;
			else if( acc_filted.x < Calibration_Acc_Min.x ) Calibration_Acc_Min.x = acc_filted.x;
			if( acc_filted.y > Calibration_Acc_Max.y ) Calibration_Acc_Max.y = acc_filted.y;
			else if( acc_filted.y < Calibration_Acc_Min.y ) Calibration_Acc_Min.y = acc_filted.y;
			if( acc_filted.z > Calibration_Acc_Max.z ) Calibration_Acc_Max.z = acc_filted.z;
			else if( acc_filted.z < Calibration_Acc_Min.z ) Calibration_Acc_Min.z = acc_filted.z;
			
			if( gyro_filted.x > Calibration_Gyro_Max.x ) Calibration_Gyro_Max.x = gyro_filted.x;
			else if( gyro_filted.x < Calibration_Gyro_Min.x ) Calibration_Gyro_Min.x = gyro_filted.x;
			if( gyro_filted.y > Calibration_Gyro_Max.y )Calibration_Gyro_Max.y = gyro_filted.y;
			else if( gyro_filted.y < Calibration_Gyro_Min.y ) Calibration_Gyro_Min.y = gyro_filted.y;
			if( gyro_filted.z > Calibration_Gyro_Max.z ) Calibration_Gyro_Max.z = gyro_filted.z;
			else if( gyro_filted.z < Calibration_Gyro_Min.z ) Calibration_Gyro_Min.z = gyro_filted.z;
			
			double acc_fluctuation_range;	double gyro_fluctuation_range;
			vector3<double> v2=Calibration_Acc_Max-Calibration_Acc_Min;
			vector3<double> v1=Calibration_Gyro_Max-Calibration_Gyro_Min;
		 
			gyro_fluctuation_range=safe_sqrt(v1.get_square());
			acc_fluctuation_range=safe_sqrt(v2.get_square());
		/*静止检测*/
		
		if( ( acc_fluctuation_range > 65 ) || ( gyro_fluctuation_range > 0.1 ) )
		{	//判断非静止
			sendLedSignal(LEDSignal_Err1);
			return MR_Err;
		}
		
		//计算校准值
		++calib_n;
		for( uint8_t i = 0; i < IMU_Sensors_Count; ++i )
		{
			if( calib_gyroscope[i] )
			{
				IMU_Sensor sensor;
				
				//陀螺
				if( GetGyroscope( i, &sensor ) )
				{
					if(sensor.have_temperature)
					{
						vector3<double> data_raw(
							sensor.data_raw.x,
							sensor.data_raw.y,
							sensor.data_raw.z
						);
						sum_gyro_t[i] += sensor.temperature;
						sum_gyro_t2[i] += sensor.temperature*sensor.temperature;
						sum_gyro_y[i] += data_raw;
						sum_gyro_ty[i] += data_raw*sensor.temperature;
						if( sensor.temperature < gyro_min_temperature[i] )
							gyro_min_temperature[i] = sensor.temperature;
						if( sensor.temperature > gyro_max_temperature[i] )
							gyro_max_temperature[i] = sensor.temperature;
					}
					else
						calib_gyroscope[i] = false;
				}
				else
					calib_gyroscope[i] = false;
				
				//加速度计
				if( GetAccelerometer( i, &sensor ) )
				{
					if(sensor.have_temperature)
					{
						vector3<double> data_raw(
							sensor.data_raw.x,
							sensor.data_raw.y,
							sensor.data_raw.z
						);
						sum_acc_t[i] += sensor.temperature;
						sum_acc_t2[i] += sensor.temperature*sensor.temperature;
						sum_acc_y[i] += data_raw;
						sum_acc_ty[i] += data_raw*sensor.temperature;
						if( sensor.temperature < acc_min_temperature[i] )
							acc_min_temperature[i] = sensor.temperature;
						if( sensor.temperature > acc_max_temperature[i] )
							acc_max_temperature[i] = sensor.temperature;
					}
					else
						calib_accelerometer[i] = false;
				}
				else
					calib_accelerometer[i] = false;
			}
		}
		
		//控制温度
		switch(calibration_step)
		{
			case 0:	//第一次升温
			{
				set_TargetIMUTemperature(60);
				if( calibration_start_time.get_pass_time() > 60 )
				{
					calibration_start_time = TIME::now();
					++calibration_step;
				}
				break;
			}
			case 1:	//第一次降温
			{
				set_TargetIMUTemperature(0);
				if( calibration_start_time.get_pass_time() > 60 )
				{
					calibration_start_time = TIME::now();
					goto CalibFinish;
				}
				break;
			}
		}
	}
	
CalibFinish:
	PR_RESULT res = PR_OK;
	double invN = 1.0/calib_n;	
	for( uint8_t i = 0; i < IMU_Sensors_Count; ++i )
	{
		if( calib_gyroscope[i] )
		{
			IMU_Sensor sensor;
			if( GetGyroscope( i, &sensor ) )
			{
				double avg_sum_t = sum_gyro_t[i] * invN;
				double avg_sum_t2 = sum_gyro_t2[i] * invN;
				double TemperatureCoefficientD = 1.0 / (avg_sum_t2 - avg_sum_t*avg_sum_t);
				vector3<double> avg_sum_gyro_y = sum_gyro_y[i]*invN;
				vector3<double> avg_sum_gyro_ty = sum_gyro_ty[i]*invN;
				vector3<double> TemperatureCoefficient = (avg_sum_gyro_ty - avg_sum_gyro_y*avg_sum_t) * TemperatureCoefficientD;
				double STTemperature = gyro_min_temperature[i];
				vector3<double> GyroOffset = avg_sum_gyro_y - TemperatureCoefficient*avg_sum_t;
				GyroOffset += TemperatureCoefficient*STTemperature;
				
				IMUConfig cfg;
				cfg.scale[0] = cfg.scale[1] = cfg.scale[2] = 1;
				cfg.offset[0] = GyroOffset.x;	cfg.offset[1] = GyroOffset.y;	cfg.offset[2] = GyroOffset.z;
				cfg.STTemperature = STTemperature;
				cfg.TemperatureCoefficient[0] = TemperatureCoefficient.x;	cfg.TemperatureCoefficient[1] = TemperatureCoefficient.y;	cfg.TemperatureCoefficient[2] = TemperatureCoefficient.z;
				if( res == PR_OK )
					res = UpdateParamGroup( sensor.name+"_Gyro", (uint64_t*)&cfg, 0, IMUConfigLength );
				else
					UpdateParamGroup( sensor.name+"_Gyro", (uint64_t*)&cfg, 0, IMUConfigLength );
			}
		}
		
		if( calib_accelerometer[i] )
		{
			IMU_Sensor sensor;
			if( GetAccelerometer( i, &sensor ) )
			{
				double avg_sum_t = sum_acc_t[i] * invN;
				double avg_sum_t2 = sum_acc_t2[i] * invN;
				double TemperatureCoefficientD = 1.0 / (avg_sum_t2 - avg_sum_t*avg_sum_t);
				vector3<double> avg_sum_acc_y = sum_acc_y[i]*invN;
				vector3<double> avg_sum_acc_ty = sum_acc_ty[i]*invN;
				vector3<double> TemperatureCoefficient = (avg_sum_acc_ty - avg_sum_acc_y*avg_sum_t) * TemperatureCoefficientD;
				double STTemperature = acc_min_temperature[i];
				vector3<double> AccOffset = avg_sum_acc_y - TemperatureCoefficient*avg_sum_t;
				AccOffset += TemperatureCoefficient*STTemperature;
								
				IMUConfig cfg;
				res = ReadParamGroup( sensor.name+"_Acc", (uint64_t*)&cfg, 0 );
				cfg.STTemperature = STTemperature;
				cfg.TemperatureCoefficient[0] = TemperatureCoefficient.x;	cfg.TemperatureCoefficient[1] = TemperatureCoefficient.y;	cfg.TemperatureCoefficient[2] = TemperatureCoefficient.z;
				if( res == PR_OK )
					res = UpdateParamGroup( sensor.name+"_Acc", (uint64_t*)&cfg, 0, IMUConfigLength );
				else
					UpdateParamGroup( sensor.name+"_Acc", (uint64_t*)&cfg, 0, IMUConfigLength );
			}
		}
	}
	
	if( res == PR_OK )
	{
		sendLedSignal(LEDSignal_Success1);
		return MR_OK;
	}
	else
	{
		sendLedSignal(LEDSignal_Err1);
		return MR_Err;
	}
}