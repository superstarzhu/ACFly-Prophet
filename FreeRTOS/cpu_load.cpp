#include "FreeRTOS.h"
#include "TimeBase.hpp"

static float CPULoad = 0;
static TIME IdleTime(false);
static TIME IdleStartTime(false);
extern "C" void EnterIdleTask()
{
	IdleStartTime = TIME::now();
}
extern "C" void ExitIdleTask()
{
	TIME passTime = TIME::now();
	passTime -= IdleStartTime;
	IdleTime += passTime;
}
extern "C" void vApplicationTickHook()
{
	static uint16_t ticks = 0;
	if( ++ticks >= configTICK_RATE_HZ/2 )
	{	//0.5秒刷新一次使用率
		ticks = 0;
		CPULoad = 100 - IdleTime.get_pass_time_fromStartUp() / 0.5f * 100;
		IdleTime.set_invalid();
	}
}

extern "C" float getCPULoad()
{
	return CPULoad;
}