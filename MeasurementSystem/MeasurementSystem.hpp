#pragma once

#include "vector2.hpp"
#include "vector3.hpp"
#include "Quaternion.hpp"
#include "map_projection.hpp"

//获取三字节WGA识别码
void MS_get_WGA( uint32_t* WGA );
//获取正版验证结果
bool MS_WGA_Correct();

enum MS_Status
{
	MS_Initializing ,
	MS_Ready ,
	MS_Err ,
};

/*健康度信息*/
	struct PosSensorHealthInf1
	{
		//传感器序号
		uint8_t sensor_ind;
		//解算位置
		vector3<double> PositionENU;
		//传感器位置
		double sensor_pos;
		//传感器偏移（传感器健康时更新）
		//HOffset+PositionENU = 传感器估计值
		double HOffset;
		//上次健康时间
		TIME last_healthy_TIME;
		//是否可用（不可用时噪声无效）
		bool available;
		//传感器噪声上下界（传感器-解算）
		double NoiseMin, NoiseMax;
		//速度噪声
		double VNoise;
	};
	struct PosSensorHealthInf2
	{
		//传感器序号
		uint8_t sensor_ind;
		//是否全球定位传感器
		//是才有定位转换信息
		bool global_sensor;
		//定位坐标转换信息
		Map_Projection mp;
		//解算位置
		vector3<double> PositionENU;
		//传感器位置
		vector2<double> sensor_pos;
		//传感器偏移（传感器健康时更新）
		//HOffset+PositionENU = 传感器估计值
		vector2<double> HOffset;
		//上次健康时间
		vector2<TIME> last_healthy_TIME;
		//是否可用（不可用时噪声无效）
		bool available;
		//传感器噪声上下界（传感器-解算）
		vector2<double> NoiseMin, NoiseMax;
		//速度噪声
		vector2<double> VNoise;
	};
	struct PosSensorHealthInf3
	{
		//传感器序号
		uint8_t sensor_ind;
		//是否全球定位传感器
		//是才有定位转换信息
		bool global_sensor;
		//定位坐标转换信息
		Map_Projection mp;
		//解算位置
		vector3<double> PositionENU;
		//传感器位置
		vector3<double> sensor_pos;
		//传感器偏移（传感器健康时更新）
		//HOffset+PositionENU = 传感器估计值
		vector3<double> HOffset;
		//上次健康时间
		vector3<TIME> last_healthy_TIME;
		//是否可用（不可用时噪声无效）
		bool available;
		//传感器噪声上下界（传感器-解算）
		vector3<double> NoiseMin, NoiseMax;
		//速度噪声
		vector3<double> VNoise;		
	};

	/*XY传感器健康度*/
		//获取当前XY传感器
		int8_t get_Current_XYSensor();
	
		//指定序号传感器健康度
		bool get_PosSensorHealth_XY( PosSensorHealthInf2* result, uint8_t sensor_ind, double TIMEOUT = -1 );
		//当前传感器健康度
		bool get_Health_XY( PosSensorHealthInf2* result, double TIMEOUT = -1 );
		//最优测距传感器健康度
		bool get_OptimalRange_XY( PosSensorHealthInf2* result, double TIMEOUT = -1 );
		//最优全球定位传感器健康度
		bool get_OptimalGlobal_XY( PosSensorHealthInf2* result, double TIMEOUT = -1 );
	/*XY传感器健康度*/
	
	/*Z传感器健康度*/
		//获取当前Z传感器
		int8_t get_Current_ZSensor();
	
		//指定序号传感器健康度
		bool get_PosSensorHealth_Z( PosSensorHealthInf1* result, uint8_t sensor_ind, double TIMEOUT = -1 );
		//当前传感器健康度
		bool get_Health_Z( PosSensorHealthInf1* result, double TIMEOUT = -1 );
		//最优测距传感器健康度
		bool get_OptimalRange_Z( PosSensorHealthInf1* result, double TIMEOUT = -1 );
		//最优全球定位传感器健康度
		bool get_OptimalGlobal_Z( PosSensorHealthInf1* result, double TIMEOUT = -1 );
	/*Z传感器健康度*/
	
	/*XYZ传感器健康度*/
		//指定序号传感器健康度
		bool get_PosSensorHealth_XYZ( PosSensorHealthInf3* result, uint8_t sensor_ind, double TIMEOUT = -1 );
		//最优测距传感器健康度
		bool get_OptimalRange_XYZ( PosSensorHealthInf3* result, double TIMEOUT = -1 );
		//最优全球定位传感器健康度
		bool get_OptimalGlobal_XYZ( PosSensorHealthInf3* result, double TIMEOUT = -1 );
	/*XYZ传感器健康度*/
/*健康度信息*/

/*姿态信息获取接口*/
	//获取解算系统状态
	MS_Status get_Attitude_MSStatus();

	//获取用于控制的滤波后的角速度
	bool get_AngularRate_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	//获取姿态四元数
	bool get_Attitude_quat( Quaternion* result, double TIMEOUT = -1 );	
	//获取机体四元数（偏航不对准）
	bool get_Airframe_quat( Quaternion* result, double TIMEOUT = -1 );
	//获取机体四元数（偏航对准）
	bool get_AirframeY_quat( Quaternion* result, double TIMEOUT = -1  );
	//获取历史四元数
	bool get_history_AttitudeQuat( Quaternion* result, double t, double TIMEOUT = -1 );
	//获取历史机体四元数（偏航不对准）
	bool get_history_AirframeQuat( Quaternion* result, double t, double TIMEOUT = -1 );
	//获取历史机体四元数（偏航对准）
	bool get_history_AirframeQuatY( Quaternion* result, double t, double TIMEOUT = -1 );
/*姿态信息获取接口*/

/*位置信息获取接口*/
	//获取解算系统状态
	MS_Status get_Altitude_MSStatus();
	MS_Status get_Position_MSStatus();
	
	//获取实时位置
	bool get_Position( vector3<double>* result, double TIMEOUT = -1 );
	//获取实时速度（东北天方向）
	bool get_VelocityENU( vector3<double>* result, double TIMEOUT = -1 );
	//获取实时地理系加速度（东北天）
	bool get_AccelerationENU( vector3<double>* result, double TIMEOUT = -1 );
	//获取用于控制的滤波后的地理系加速度
	bool get_AccelerationENU_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	bool get_VelocityENU_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
	bool get_Position_Ctrl( vector3<double>* result, double TIMEOUT = -1 );
/*位置信息获取接口*/

/*电池信息接口*/
	struct BatteryCfg
	{
		//标准电压（V）
		float STVoltage[2];
		//电压测量增益（V）
		float VoltMKp[2];
		//电流测量增益（A）
		float CurrentMKp[2];
		//容量（W*h）
		float Capacity[2];
		//功率电压点0（0%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP0[2];
		//功率电压点1（10%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP1[2];
		//功率电压点2（20%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP2[2];
		//功率电压点3（30%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP3[2];
		//功率电压点4（40%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP4[2];
		//功率电压点5（50%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP5[2];
		//功率电压点6（60%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP6[2];
		//功率电压点7（70%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP7[2];
		//功率电压点8（80%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP8[2];
		//功率电压点9（90%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP9[2];
		//功率电压点10（100%电量时相对标准电压的电压差，此序列必须递增）
		float VoltP10[2];
	}__PACKED;
	
	//电压
	float get_MainBatteryVoltage();
	//滤波电压（V）
	float get_MainBatteryVoltage_filted();
	//总使用功耗（W*h）
	float get_MainBatteryPowerUsage();
	//滤波功率（W）
	float get_MainBatteryPower_filted();
	//电池电流（A）
	float get_MainBatteryCurrent();
	//CPU温度（℃）
	float get_CPUTemerature();
	//获取电池信息
	void get_MainBatteryInf( float* Volt, float* Current, float* PowerUsage, float* Power_filted, float* RMPercent );
/*电池信息接口*/
