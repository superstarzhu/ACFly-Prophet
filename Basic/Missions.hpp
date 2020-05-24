#pragma once

#include "Basic.hpp"

struct MissionInf
{
	uint16_t cmd;
	uint8_t frame;
	uint8_t autocontinue;
	uint8_t rsv[4];
	double params[7];
}__attribute__((__packed__));


/*
	设置当前任务
	wpInd：当前任务序号
*/
bool setCurrentMission( uint16_t wpInd );
/*
	读取当前任务序号
*/
uint16_t getCurrentMissionInd();

/*
	获取航点任务个数
*/
uint16_t getMissionsCount();
/*
	获取正在上传航点个数
*/
uint16_t getUploadingMissionsCount();

/*
	清除所有航点任务
	TIMEOUT: 线程同步超时时间

	返回值：
	true:添加成功
	false:添加失败（航点过多或航点信息错误或参数表错误）
*/
bool clearMissions( double TIMEOUT = -1 );

/*
	添加航点任务
	wp_inf：航点信息
	st：是否写入存储器（只有当前实际航点数量为0才可以缓存不写入存储器）
	TIMEOUT: 线程同步超时时间

	返回值：
	true:添加成功
	false:添加失败（航点过多或航点信息错误或参数表错误）
*/
bool addMission( MissionInf wp_inf, bool st = true, double TIMEOUT = -1 );

/*
	保存航点任务
	TIMEOUT: 线程同步超时时间

	返回值：
	true:保存成功
	false:保存失败（超时）
*/
bool saveMissions( double TIMEOUT = -1 );

/*
	读取航点任务
	wp_ind：航点序号
	wp_inf：获取的航点信息
	TIMEOUT: 线程同步超时时间

	返回值：
	true:获取成功
	false:获取失败（无航点或参数表错误）
*/
bool ReadMission( uint16_t wp_ind, MissionInf* wp_inf, double TIMEOUT = -1 );

/*
	读取当前航点任务
	wp_inf：获取的航点信息
	ind：当前航点序号
	TIMEOUT: 线程同步超时时间

	返回值：
	true:获取成功
	false:获取失败（无航点或参数表错误）
*/
bool ReadCurrentMission( MissionInf* wp_inf, uint16_t* ind, double TIMEOUT = -1 );

void init_Missions();