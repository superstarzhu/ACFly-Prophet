#include "drv_Main.hpp"

#include "SensorsBackend.hpp"
#include "drv_LED.hpp"
#include "drv_Oled.hpp"
#include "GUI.hpp"
#include "drv_USB.hpp"
#include "drv_Uart1.hpp"
#include "drv_Uart3.hpp"
#include "drv_Uart5.hpp"
#include "drv_Uart7.hpp"
#include "drv_Uart8.hpp"
#include "Commulink.hpp"
#include "drv_SDMMC.hpp"
#include "drv_Sensors.hpp"
#include "drv_Ultrasonic.hpp"
#include "drv_Flash.hpp"
#include "drv_RCSbus.hpp"
#include "drv_RCPPM.hpp"
#include "drv_PWMOut.hpp"
#include "drv_ADC.hpp"

#include "StorageSystem.hpp"
#include "Parameters.hpp"

#include "drv_ExtIIC.hpp"
#include "drv_GPS.hpp"
#include "drv_ExtMag.hpp"
#include "drv_OpticalFlow.hpp"

//可选外设
#include "drv_TFMini.hpp"

#include "drv_SDI.hpp"

void init_drv_Main()
{	
	//通信外设驱动
	init_drv_LED();
	//传感器接口初始化
	init_Sensors();
	init_Commulink();
	init_drv_Oled();
	init_GUI();
	os_delay(0.1);
	init_drv_USB();	
	init_drv_Uart1();
	init_drv_Uart3();
	init_drv_Uart5();
	init_drv_Uart7();
	init_drv_Uart8();
	
	//存储外设驱动	
	init_drv_Flash();
	init_drv_SDMMC();
	init_drv_PWMOut();
//	PWM_PullUpAll();
//	os_delay(4.0);
//	PWM_PullDownAll();
	
	//ADC初始化
	init_drv_ADC();
	
	//内部存储介质初始化
	init_InternalStorage();	
	//参数表初始化
	init_Parameters();
	//SD存储初始化
	init_SDStorage();
	
	//传感器初始化
	init_drv_Sensors();
	init_drv_RCSbus();
	init_drv_RCPPM();
	
	//外部传感器初始化
	init_drv_ExtIIC();
	init_drv_ExtMag();
	init_drv_GPS();
	init_drv_ultrasonic();
	init_drv_OpticalFlow();
	
	//可选外设
	init_drv_TFMini();
	
	init_drv_SDI();
}