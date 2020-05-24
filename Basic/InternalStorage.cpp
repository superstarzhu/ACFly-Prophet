#include "StorageSystem.hpp"

#include "drv_ADC.hpp"
#include "fatfs.h"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

//是否已初始化
static bool InternalStorage_Initialized = false;
//内部存储是否第一次运行（开机时初始化）
static bool InternalStorage_FirstTime = false;

//设备访问互斥锁
static SemaphoreHandle_t InternalStorage_Semphr;
//文件
static char UserFile_filename[256];
static FIL UserFile;

/*文件读写*/
	/*
		保存存储文件
		group_name：文件组别名称
		name：文件名称
		content：内容
		length：内容长度
		TIMEOUT：线程同步超时时间
	*/
	SS_RESULT InternalStorage_SaveFile( const char* group_name, const char* name , const void* content , uint32_t length , double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//获取互斥锁
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//获取文件名
			for( uint8_t i = 0 ; i < 4 ; ++i )
				UserFile_filename[i] = FlashPath[i];
			strcat( UserFile_filename , group_name );
			//创建组别
			if( f_stat( UserFile_filename, 0 ) == FR_NO_FILE )
				f_mkdir(UserFile_filename);
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			FRESULT res = f_open(&UserFile, UserFile_filename, FA_CREATE_ALWAYS | FA_WRITE);
			//打开文件
			if(res == FR_OK) 
			{	//写入文件
				uint32_t byteswritten;
				res = f_write(&UserFile, content, length, &byteswritten);
				if( byteswritten < length )
					res = FR_DENIED;
				res = f_close(&UserFile);
			}
			
			//释放互斥锁
			xSemaphoreGive(InternalStorage_Semphr);
			if( res == FR_OK )
				return SS_OK;
			else
				return SS_ERR;
		}
		else
			return SS_TimeOut;
	}
	
	/*
		读取存储文件大小
		group_name：文件组别名称
		name：文件名称
		size：读取的文件大小
		TIMEOUT：线程同步超时时间
	*/
	SS_RESULT InternalStorage_GetFileSize( const char* group_name, const char* name , uint32_t* size , double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//获取互斥锁
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//获取文件名
			for( uint8_t i = 0 ; i < 4 ; ++i )
				UserFile_filename[i] = FlashPath[i];
			strcat( UserFile_filename , group_name );
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			FILINFO info;
			FRESULT res = f_stat( UserFile_filename, &info );
			if( res == FR_OK )
				*size = info.fsize;
			
			//释放互斥锁
			xSemaphoreGive(InternalStorage_Semphr);
			if( res == FR_OK )
				return SS_OK;
			else
				return SS_ERR;
		}
		else
			return SS_TimeOut;
	}
	/*
		读取存储文件
		group_name：文件组别名称
		name：文件名称
		content：读取的内容
		length：读取的文件长度
		TIMEOUT：线程同步超时时间
	*/
	SS_RESULT InternalStorage_ReadFile( const char* group_name, const char* name , void* content , uint32_t* length , double TIMEOUT )
	{
		if( InternalStorage_Initialized == false )
			return SS_ERR;
		
		//获取互斥锁
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( InternalStorage_Semphr , TIMEOUT_Ticks ) )
		{
			//获取文件名
			for( uint8_t i = 0 ; i < 4 ; ++i )
				UserFile_filename[i] = FlashPath[i];
			strcat( UserFile_filename , group_name );
			strcat( UserFile_filename , "/" );
			strcat( UserFile_filename , name );
			strcat( UserFile_filename , ".cfg" );
			
			FRESULT res = f_open(&UserFile, UserFile_filename, FA_READ);
			//打开文件
			if(res == FR_OK) 
			{	//读取文件
				uint32_t bytesread;
				uint32_t file_size = f_size(&UserFile);
				*length = file_size;
				res = f_read(&UserFile, content, file_size, &bytesread);
				if( bytesread < file_size )
					res = FR_DENIED;
				res = f_close(&UserFile);
			}
			
			//释放互斥锁
			xSemaphoreGive(InternalStorage_Semphr);
			if( res == FR_OK )
				return SS_OK;
			else
				return SS_ERR;
		}
		else
			return SS_TimeOut;
	}
/*文件读写*/

void init_InternalStorage()
{
	while( Get_VDDA_Voltage() < 3.0 )
	{	//电压过低不初始化flash防止误擦除
		os_delay(0.1);
	}
	
	FRESULT res;
	Aligned_DMABuf uint8_t workBuffer[FF_MAX_SS*2];
	//挂载文件系统
	//res = f_mkfs(FlashPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));
	res = f_mount(&FlashFatFS, (TCHAR const*)FlashPath, 1);
	if( res == FR_NO_FILESYSTEM )
	{	//无文件系统先格式化
		MKFS_PARM mkfs_param;
		mkfs_param.fmt = FM_FAT;
		mkfs_param.au_size = 0;
		mkfs_param.align = 0;
		mkfs_param.n_fat = 2;
		mkfs_param.n_root = 0;
 		res = f_mkfs(FlashPath, &mkfs_param, workBuffer, sizeof(workBuffer));
		InternalStorage_FirstTime = true;
	}
	
	//新建Config目录
	char name[50] = {0};
	strcat( name, FlashPath );
	strcat( name, "Config" );
	if( f_stat( name, 0 ) == FR_NO_FILE )
		f_mkdir(name);
	
	//新建Log目录
	name[0] = 0;
	strcat( name, FlashPath );
	strcat( name, "Log" );
	if( f_stat( name, 0 ) == FR_NO_FILE )
		f_mkdir(name);
	
	//更新是否已初始化状态
	if( res == FR_OK )
	{
		InternalStorage_Semphr = xSemaphoreCreateMutex();
		InternalStorage_Initialized = true;
	}
}