#pragma once

#include "Basic.hpp"
#include "Sensors.hpp"

void init_MS_Attitude();

void MS_Attitude_GyroIntegral( uint8_t ind , IMU_Sensor Gyroscope );
void MS_Attitude( uint8_t ind , IMU_Sensor Accelerometer );