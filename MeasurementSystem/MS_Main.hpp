#pragma once

#include "FreeRTOS.h"

/*参数*/
	//水平校准四元数
	struct AirframeCalibQuat
	{
		float q0,q1,q2,q3;
	}__attribute__((__packed__));
/*参数*/

/*IMU更新时间定义*/
	extern double IMU_Accelerometer_UpdateT[];
	extern double IMU_Accelerometer_UpdateFreq[];
	void set_IMU_Accelerometer_UpdateFreq( uint8_t ind , double Freq );
	
	extern double IMU_Gyroscope_UpdateT[];
	extern double IMU_Gyroscope_UpdateFreq[];
	void set_IMU_Gyroscope_UpdateFreq( uint8_t ind , double Freq );
/*IMU更新时间定义*/

/*IMU（陀螺加速度）更新通知*/
	//陀螺更新通知
	bool MS_Notify_IMUGyroUpdate( uint8_t ind );
	bool MS_Notify_IMUGyroUpdateFromISR( uint8_t ind , BaseType_t *pxHigherPriorityTaskWoken );
	
	//加速度更新通知
	bool MS_Notify_IMUAceelUpdate( uint8_t ind );
	bool MS_Notify_IMUAccelUpdateFromISR( uint8_t ind , BaseType_t *pxHigherPriorityTaskWoken );
/*IMU（陀螺加速度）更新通知*/

void init_MS_Main();