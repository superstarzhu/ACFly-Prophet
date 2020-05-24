#include "Basic.hpp"
#include "TimeBase.hpp"
#include "stm32h7xx_hal.h"

//初始化完成指示
//初始化完成不能进行初始化操作
static uint16_t InitializationStatus_Lock = 0;
static bool InitializationCompleted = false;
bool getInitializationCompleted(){ return InitializationCompleted; }
void setInitializationCompleted()
{ 
	if(InitializationStatus_Lock == 0)
		InitializationCompleted = true; 
}
void LockInitializationStatus(){ ++InitializationStatus_Lock; }
void UnLockInitializationStatus(){ --InitializationStatus_Lock; }

void init_Basic()
{	
	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();
  /* Enable D-Cache---------------------------------------------------------*/
	#ifdef DCACHE_SIZE
		SCB_EnableDCache();
	#endif
	
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	init_TimeBase();
	
	/*MPU Region 3用于AXI设置非Cache区域用于DMA传输*/
		MPU_Region_InitTypeDef MPU_Initure;		
		HAL_MPU_Disable();         //配置MPU之前先关闭MPU,配置完成以后在使能MPU
	
		MPU_Initure.Enable           = MPU_REGION_ENABLE;               //使能该保护区域
		MPU_Initure.Number           = MPU_REGION_NUMBER3;             //设置保护区域
		MPU_Initure.BaseAddress      = 0x24070000;                      //设置基址
		MPU_Initure.Size             = MPU_REGION_SIZE_64KB;            //设置保护区域大小
		MPU_Initure.SubRegionDisable = 0;                               //禁止子区域
		MPU_Initure.TypeExtField     = MPU_TEX_LEVEL1;                  //设置类型扩展域为level1
		MPU_Initure.AccessPermission = MPU_REGION_FULL_ACCESS;          //设置访问权限,
		MPU_Initure.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;   //允许指令访问(允许读取指令)
		MPU_Initure.IsShareable      = MPU_ACCESS_SHAREABLE;            //是否允许共用
		MPU_Initure.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;        //是否允许cache
		MPU_Initure.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;           //是否允许缓冲
		HAL_MPU_ConfigRegion(&MPU_Initure);                             //配置MPU
		HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);                         //开启MPU
	/*MPU Region 3用于AXI设置非Cache区域用于DMA传输*/
	
	/*MPU Region 4用于AXI设置非Cache区域用于DMA传输*/
		HAL_MPU_Disable();         //配置MPU之前先关闭MPU,配置完成以后在使能MPU
	
		MPU_Initure.Enable           = MPU_REGION_ENABLE;               //使能该保护区域
		MPU_Initure.Number           = MPU_REGION_NUMBER4;             //设置保护区域
		MPU_Initure.BaseAddress      = 0x30000000;                      //设置基址
		MPU_Initure.Size             = MPU_REGION_SIZE_128KB;            //设置保护区域大小
		MPU_Initure.SubRegionDisable = 0;                               //禁止子区域
		MPU_Initure.TypeExtField     = MPU_TEX_LEVEL1;                  //设置类型扩展域为level1
		MPU_Initure.AccessPermission = MPU_REGION_FULL_ACCESS;          //设置访问权限,
		MPU_Initure.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;   //允许指令访问(允许读取指令)
		MPU_Initure.IsShareable      = MPU_ACCESS_SHAREABLE;            //是否允许共用
		MPU_Initure.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;        //是否允许cache
		MPU_Initure.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;           //是否允许缓冲
		HAL_MPU_ConfigRegion(&MPU_Initure);                             //配置MPU
		HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);                         //开启MPU
	/*MPU Region 4用于AXI设置非Cache区域用于DMA传输*/
}

/*重写C++ new delete运算符*/
	void * operator new( std::size_t size )
	{
		return pvPortMalloc( size );
	}
	void * operator new[]( std::size_t size )
	{
		return pvPortMalloc(size);
	}

	void  operator delete(void* __p) _NOEXCEPT
	{
		return vPortFree ( __p );
	}
	void  operator delete[](void* __p) _NOEXCEPT
	{
		return vPortFree ( __p );
	}
/*重写C++ new delete运算符*/