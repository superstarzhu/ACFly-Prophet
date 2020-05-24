#include "Basic.hpp"
#include "drv_ExtIIC.hpp"
#include "SensorsBackend.hpp"

/*外置罗盘定义*/
	struct External_MagnetoMeter
	{
		//传感器名称
		SName name;
		
		//iic地址
		unsigned char device_address;
		//数据寄存器地址
		unsigned char data_address;
		
		//是否高位在前
		bool MSB;
		//是否需要手动开始采样
		bool need_set_sample;
		//采样寄存器地址
		unsigned char sample_address;
		//采样寄存器值
		unsigned char sample_cfg;
		
		//ID寄存器地址
		unsigned char ID_address;
		//ID
		unsigned char ID;
		
		//配置寄存器地址
		unsigned char configurations_addresses[10];
		//配置数目
		unsigned char configuration_count;
		//配置
		unsigned char configurations[10];
		//灵敏度（数据->Gauss）
		double sensitivity;
		
		//采样率（多少ms一次数据）
		unsigned char sample_mseconds;
		
		//轴向顺序
		//从1开始，负数为反向
		//如：1,-3,2代表轴向为 x , -z , y
		signed char axis_index[3];
	};
		
	static const External_MagnetoMeter External_MagnetoMeters[] = 
	{
	/*     名称     , iic地址, 数据寄存器地址 , 高位在前, 手动采样 , 采样寄存器地址, 采样设置, ID 地址 , ID   ,      配置寄存器地址      , 配置数目,  配置                                                     ,  灵敏度      , 采样率毫秒  , 轴向                 ,  */
		{ "HMC5983"   , 0x1e   ,      3         ,    true ,   false  ,            0  ,      0  ,      0  , 0x10 ,      { 0, 1, 2 }         , 3       , { (1<<7) | (0<<5) | (7<<2) | (0<<0) , (1<<5) , 0 }        , 0.00092      , 5           , { -3 , -1 , -2 }   } , //HMC5983
		{ "HMC5883"   , 0x1e   ,      3         ,    true ,   false  ,            0  ,      0  ,      0  , 0xf0 ,      { 0, 1, 2 }         , 3       , { (0<<7) | (0<<5) | (6<<2) | (0<<0) , (1<<5) , 0 }        , 0.00092      , 20          , { -3 , -1 , -2 }   } , //HMC5883
		{ "QMC5883"   , 0x0d   ,      0         ,   false ,   false  ,            0  ,      0  ,      9  , 0x00 ,      { 0x9, 0xa, 0xb }   , 3       , { (0<<6) | (0<<4) | (0b10<<2) | (1<<0) , (1<<6) , 1 }     , 8.33333333e-5, 10          , { -2 , -1 , -3 }   } , //HMC5883
		{ "Mag3110"   , 0x0e   ,      1         ,    true ,   false  ,            0  ,      0  ,      7  , 0xc4 ,      { 0x10 }            , 1       , { (0<<5) | (0<<3) | (0<<2) | (0<<1) | (1<<0) }            , 0.001        , 10          , { -2 , -1 , 3 }    } , //Mag3110
		{ "IST8310"   , 0x0e   ,      3         ,   false ,    true  ,          0xa  ,      1  ,      0  , 0x10 ,      { 0x41, 0x42 }      , 2       , { 0x24 , 0xc0 }                                           , 0.003        , 20          , { -1 , -2 , -3 }    } , //IST8310
	};
	static const uint8_t Supported_External_MagnetoMeter_Count = sizeof( External_MagnetoMeters ) / sizeof( External_MagnetoMeter );
/*外置罗盘定义*/


static void ExtMag_Server(void* pvParameters)
{
ScanExtMag:
	//当前使用的外置罗盘型号
	int8_t current_ExtMag = -1;
	//缓冲区
	Static_AXIDMABuf uint8_t tx_buf[12];
	__attribute__ ((aligned (4))) Static_AXIDMABuf uint8_t rx_buf[32];
	
	//操作结果
	bool res;
	//准确周期延时
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, 0.1*configTICK_RATE_HZ );
		for( int8_t i = 0; i < Supported_External_MagnetoMeter_Count; ++i )
		{
			//判断传感器ID是否正确
			tx_buf[0] = External_MagnetoMeters[i].ID_address;
			res = ExtIIC_SendReceiveAddr7( External_MagnetoMeters[i].device_address, tx_buf, 1, rx_buf, 1 );
			if(!res)
				continue;
			if( rx_buf[0] != External_MagnetoMeters[i].ID )
				continue;
			
			//发送传感器配置			
			for( uint8_t k = 0; k < External_MagnetoMeters[i].configuration_count; ++k )
			{
				tx_buf[0] = External_MagnetoMeters[i].configurations_addresses[k];
				tx_buf[1] = External_MagnetoMeters[i].configurations[k];
				res = ExtIIC_SendAddr7( External_MagnetoMeters[i].device_address, tx_buf, 2 );
				if(!res)
					break;
			}			
			if(!res)
				continue;
			
			//检查传感器配置是否成功
			for( uint8_t k = 0; k < External_MagnetoMeters[i].configuration_count; ++k )
			{
				tx_buf[0] = External_MagnetoMeters[i].configurations_addresses[k];
				res = ExtIIC_SendReceiveAddr7( External_MagnetoMeters[i].device_address, tx_buf, 1, rx_buf, 1 );
				if(!res)
					break;
				if( rx_buf[0] != External_MagnetoMeters[i].configurations[k] )
				{	//配置校验错误
					res = false;
					break;
				}
			}
			if(!res)
				continue;
			
			//已成功识别到传感器
			if( IMUMagnetometerRegister( External_Magnetometer_Index, External_MagnetoMeters[i].name, External_MagnetoMeters[i].sensitivity ) )
			{
				current_ExtMag = i;
				goto ExtMagDetected;
			}
		}
	}
	
ExtMagDetected:
	const External_MagnetoMeter* sensor = &External_MagnetoMeters[current_ExtMag];
	while(1)
	{
		//周期对传感器采样
		vTaskDelay( sensor->sample_mseconds*1e-3*configTICK_RATE_HZ+1 );
		
		//采样
		uint8_t rt = 0;
		do
		{
			//失败次数过多重新扫描
			if( ++rt > 3 )
			{
				IMUMagnetometerUnRegister(External_Magnetometer_Index);
				goto ScanExtMag;
			}
			//读取数据
			tx_buf[0] = sensor->data_address;
			res = ExtIIC_SendReceiveAddr7( sensor->device_address, tx_buf, 1, rx_buf, 6 );
			if( sensor->need_set_sample )
			{	//需要发送采样指令
				tx_buf[0] = sensor->sample_address;
				tx_buf[1] = sensor->sample_cfg;
				res &= ExtIIC_SendAddr7( sensor->device_address, tx_buf, 2 );
			}
		}while(res==false);
		
		/*更新传感器数据*/						
			//大端转小端
			if( sensor->MSB )
			{
				((uint16_t*)&rx_buf)[0] = __REV16( ((uint16_t*)&rx_buf)[0] );
				((uint16_t*)&rx_buf)[1] = __REV16( ((uint16_t*)&rx_buf)[1] );
				((uint16_t*)&rx_buf)[2] = __REV16( ((uint16_t*)&rx_buf)[2] );
			}			
			
			//轴向转换
			vector3<int32_t> data;
			if( sensor->axis_index[0] > 0 )
				data.x = (signed short)((uint16_t*)&rx_buf)[ sensor->axis_index[0] - 1 ];
			else
				data.x = -(signed short)((uint16_t*)&rx_buf)[ (-sensor->axis_index[0]) - 1 ];
			if( sensor->axis_index[1] > 0 )
				data.y = (signed short)((uint16_t*)&rx_buf)[ sensor->axis_index[1] - 1 ];
			else
				data.y = -(signed short)((uint16_t*)&rx_buf)[ (-sensor->axis_index[1]) - 1 ];
			if( sensor->axis_index[2] > 0 )
				data.z = (signed short)((uint16_t*)&rx_buf)[ sensor->axis_index[2] - 1 ];
			else
				data.z = -(signed short)((uint16_t*)&rx_buf)[ (-sensor->axis_index[2]) - 1 ];
				
			//更新数据
			IMUMagnetometerUpdate( External_Magnetometer_Index, data, false);
		/*更新传感器数据*/
	}
}

void init_drv_ExtMag()
{
	xTaskCreate( ExtMag_Server, "ExtMag", 1024, NULL, SysPriority_ExtSensor, NULL);
}