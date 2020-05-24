#include "Missions.hpp"

#include "Parameters.hpp"
#include "semphr.h"

#define MissionParamVersion 1
#define MaxMissions 512
#define MissionsInParamGroupBit 5
#define MissionsInParamGroup (1<<MissionsInParamGroupBit)

static SemaphoreHandle_t MissionsSemphr = xSemaphoreCreateMutex();

static inline bool Lock_Missions( double TIMEOUT = -1 )
{
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( MissionsSemphr , TIMEOUT_Ticks ) )
		return true;
	return false;
}
static inline void UnLock_Missions()
{
	xSemaphoreGive(MissionsSemphr);
}

//航点个数
static uint16_t MissionsCount = 0;
static uint16_t UploadingMissionsCount = 0;

//当前航点
static uint16_t CurrentMission = 0;
/*
	设置当前任务
	wpInd：当前任务序号
*/
bool setCurrentMission( uint16_t wpInd )
{
	if( MissionsCount==0 && wpInd==0 )
	{
		CurrentMission = 0;
		return false;
	}
	if( wpInd >= MissionsCount )
		return false;
	CurrentMission = wpInd;
	return true;
}
/*
	读取当前任务序号
*/
uint16_t getCurrentMissionInd() { return CurrentMission; }

/*
	获取航点个数
*/
uint16_t getMissionsCount()
{
	return MissionsCount;
}
/*
	获取正在上传航点个数
*/
uint16_t getUploadingMissionsCount()
{
	return UploadingMissionsCount;
}

/*
	清除所有航点任务
	TIMEOUT: 线程同步超时时间

	返回值：
	true:添加成功
	false:添加失败（航点过多或航点信息错误或参数表错误）
*/
bool clearMissions( double TIMEOUT )
{
	if( Lock_Missions(TIMEOUT) )
	{
		MissionInf wp_inf;
		wp_inf.cmd = 0;
		UpdateParamGroup( "Mission_0", (uint64_t*)&wp_inf, 0*8, 8 );
		
		CurrentMission = MissionsCount = UploadingMissionsCount = 0;
		
		UnLock_Missions();
		return true;
	}
	return false;
}

/*
	添加航点任务
	wp_inf：航点信息
	st：是否写入存储器（只有当前实际航点数量为0才可以缓存不写入存储器）
	TIMEOUT: 线程同步超时时间

	返回值：
	true:添加成功
	false:添加失败（航点过多或航点信息错误或参数表错误）
*/
bool addMission( MissionInf wp_inf, bool st, double TIMEOUT )
{
	if( wp_inf.cmd == 0 )
		return false;
	if( MissionsCount > MaxMissions )
		return false;
	if( st==false && MissionsCount>0 )
		return false;
	if( st && MissionsCount!=UploadingMissionsCount )
		return false;
	
	if( Lock_Missions(TIMEOUT) )
	{
		//注册参数组
		uint16_t ParamGroupInd = UploadingMissionsCount >> MissionsInParamGroupBit;
		char WPGroupName[17];
		sprintf( WPGroupName, "Mission_%d", ParamGroupInd );
		MissionInf wp_infs[MissionsInParamGroup];
		for( uint8_t i = 0; i < MissionsInParamGroup; ++i )
			wp_infs[i].cmd = 0;
		PR_RESULT res = ParamGroupRegister( SName(WPGroupName), MissionParamVersion, MissionsInParamGroup*8, 0, 0, (uint64_t*)wp_infs );
		if( res == PR_ERR )
			return false;
		
		MissionInf st_infs[2];
		st_infs[0] = wp_inf;
		st_infs[1].cmd = 0;
		
		//更改当前航点
		uint16_t wpInd = UploadingMissionsCount & (MissionsInParamGroup-1);
		if( wpInd < MissionsInParamGroup - 1 )
			UpdateParamGroup( SName(WPGroupName), (uint64_t*)&st_infs[0], wpInd*8, 8*2, st );
		else
		{
			UpdateParamGroup( SName(WPGroupName), (uint64_t*)&st_infs[0], wpInd*8, 8, st );
			//将下一个航点的cmd改为0
			sprintf( WPGroupName, "Mission_%d", ParamGroupInd+1 );
			UpdateParamGroup( SName(WPGroupName), (uint64_t*)&st_infs[1], 0*8, 8, st );
		}

		if(st)
		{
			++MissionsCount;
			UploadingMissionsCount = MissionsCount;
		}
		else
			++UploadingMissionsCount;
		
		UnLock_Missions();
		return true;
	}
	return false;
}

/*
	保存航点任务
	TIMEOUT: 线程同步超时时间

	返回值：
	true:保存成功
	false:保存失败（超时）
*/
bool saveMissions( double TIMEOUT )
{
	if( UploadingMissionsCount == 0 )
		return true;
	if( Lock_Missions(TIMEOUT) )
	{
		uint16_t ParamGroupInd = (UploadingMissionsCount-1) >> MissionsInParamGroupBit;
		for( uint16_t i = 0; i <= ParamGroupInd; ++i )
		{
			char WPGroupName[17];
			sprintf( WPGroupName, "Mission_%d", i );
			SaveParamGroup( WPGroupName );
		}
		MissionsCount = UploadingMissionsCount;
		
		UnLock_Missions();
		return true;
	}
	return false;
}

/*
	读取航点任务
	wp_ind：航点序号
	wp_inf：获取的航点信息
	TIMEOUT: 线程同步超时时间

	返回值：
	true:获取成功
	false:获取失败（无航点或参数表错误）
*/
bool ReadMission( uint16_t wp_ind, MissionInf* wp_inf, double TIMEOUT )
{
	if( wp_ind >= MissionsCount )
		return false;
	
	uint16_t ParamGroupInd = wp_ind >> MissionsInParamGroupBit;
	uint16_t wpInd = wp_ind & (MissionsInParamGroup-1);
	char WPGroupName[17];
	sprintf( WPGroupName, "Mission_%d", ParamGroupInd );
	PR_RESULT res = ReadParamGroup( SName(WPGroupName), (uint64_t*)wp_inf, 0, wpInd*8, 8 );
	if( res == PR_ERR )
		return false;
	if( wp_inf->cmd == 0 )
		return false;
	return true;
}

/*
	读取当前航点任务
	wp_inf：获取的航点信息
	ind：当前航点序号
	TIMEOUT: 线程同步超时时间

	返回值：
	true:获取成功
	false:获取失败（无航点或参数表错误）
*/
bool ReadCurrentMission( MissionInf* wp_inf, uint16_t* ind, double TIMEOUT )
{
	if( MissionsCount == 0 )
		return false;
	if( CurrentMission >= MissionsCount )
		return false;
	
	uint16_t ParamGroupInd = CurrentMission >> MissionsInParamGroupBit;
	uint16_t wpInd = CurrentMission & (MissionsInParamGroup-1);
	char WPGroupName[17];
	sprintf( WPGroupName, "Mission_%d", ParamGroupInd );
	PR_RESULT res = ReadParamGroup( SName(WPGroupName), (uint64_t*)wp_inf, 0, wpInd*8, 8 );
	if( res == PR_ERR )
		return false;
	if( wp_inf->cmd == 0 )
		return false;
	if(ind!=0)
		*ind = CurrentMission;
	return true;
}

void init_Missions()
{	
	char WPGroupName[17];	
	MissionsCount = UploadingMissionsCount = 0;
	
	while(1)
	{
		uint16_t ParamGroupInd = UploadingMissionsCount >> MissionsInParamGroupBit;
		sprintf( WPGroupName, "Mission_%d", ParamGroupInd );	
		
		PR_RESULT res = ParamGroupRegister( SName(WPGroupName), MissionParamVersion, MissionsInParamGroup*8, 0 );
		if( res == PR_ERR )
			break;  
		
		for( uint8_t i = 0; i < MissionsInParamGroup; ++i )
		{
			MissionInf wp_inf;
			res = ReadParamGroup( SName(WPGroupName), (uint64_t*)&wp_inf, 0, i*8, 8 );
			if( res == PR_ERR )
				return;
			if( wp_inf.cmd == 0 )
				return;
			
			++MissionsCount;
			UploadingMissionsCount = MissionsCount;
			if( MissionsCount > MaxMissions )
				return;
		}		
	}
}