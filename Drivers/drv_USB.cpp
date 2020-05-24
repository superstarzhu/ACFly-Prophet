#include "drv_USB.hpp"
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "Basic.hpp"
#include "CommuLink.hpp"

//USB设备定义
USBD_HandleTypeDef hUsbDeviceFS;

//USB发送互斥锁
static SemaphoreHandle_t USBD_VCOM_TxSemphr;
//USB发送消息缓冲区
#define TxStreamBufferSize 1024
#define TxBufferSize 256
static uint8_t USBD_VCOM_TxBuffer[TxBufferSize];
static StreamBufferHandle_t USBD_VCOM_TxStreamBuffer;

//USB接收互斥锁
static SemaphoreHandle_t USBD_VCOM_RxSemphr;
//USB接收缓冲区
#define RxStreamBufferSize 1024
static StreamBufferHandle_t USBD_VCOM_RxStreamBuffer;

/*发送上锁
	上锁保证数据连续性
	上锁之后必须解锁
*/
bool Lock_USBD_VCOM( double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTicks > 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	if( xSemaphoreTakeRecursive( USBD_VCOM_TxSemphr , Sync_waitTicks ) == pdTRUE )
		return true;
	return false;
}
void Unlock_USBD_VCOM()
{
	xSemaphoreGiveRecursive( USBD_VCOM_TxSemphr );
}

/*USBD虚拟串口发送函数（将要发送的数据压入缓冲区）
	data:要发送的数据指针
	length:要发送的数据长度（字节）
	Sync_waitTime:线程同步最大等待时间（s）
	Send_waitTime:等待缓冲区有空间的最大等待时间（s）
	返回值：实际发送的字节数
	解释：如果在指定等待时间内接收不到足够数据（发送缓冲区位置不足）
				将只发送前半段数据（只将前半段数据压入缓冲区）
*/
uint16_t Write_USBD_VCOM( const uint8_t *data, uint16_t length, double Send_waitTime, double Sync_waitTime )
{
	if( length == 0 )
		return 0;
	uint32_t Sync_waitTicks;
	if( Sync_waitTicks >= 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	uint32_t Send_waitTicks;
	if( Send_waitTime >= 0 )
		Send_waitTicks = Send_waitTime*configTICK_RATE_HZ;
	else
		Send_waitTicks = portMAX_DELAY;
	//获取信号量
	if( xSemaphoreTakeRecursive( USBD_VCOM_TxSemphr , Sync_waitTicks ) == pdTRUE )
	{	
		uint16_t data_sent = xStreamBufferSend( USBD_VCOM_TxStreamBuffer , data , length , Send_waitTicks );
		USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
		if( hcdc->TxState == 0 )
		{
			//USB空闲可发送
			uint16_t sd_length;
			sd_length = xStreamBufferReceive( USBD_VCOM_TxStreamBuffer , USBD_VCOM_TxBuffer , TxBufferSize , 0 );
			if( sd_length > 0 )
				CDC_Transmit_FS( USBD_VCOM_TxBuffer , sd_length );
		}
		//释放信号量
		xSemaphoreGiveRecursive( USBD_VCOM_TxSemphr );
		return data_sent;
	}
	else
		return 0;
}

/*USBD虚拟串口接收函数（将要发送的数据压入缓冲区）
	data:接收数据指针
	length:要接收的数据长度（字节）
	Sync_waitTime:线程同步最大等待时间（s）
	Rc_waitTime:等待数据的最大等待时间（s）
	返回值：实际接收到的字节数
	解释:如果在指定等待时间内接收不到足够数据（接收缓冲区里没这么多数据）
				就接收尽量多的数据
*/
uint16_t Read_USBD_VCOM( uint8_t *data, uint16_t length, double Rc_waitTime, double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTime >= 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	uint32_t Rc_waitTicks;
	if( Rc_waitTime >= 0 )
		Rc_waitTicks = Rc_waitTime*configTICK_RATE_HZ;
	else
		Rc_waitTicks = portMAX_DELAY;
	//获取信号量
	if( xSemaphoreTake( USBD_VCOM_RxSemphr , Sync_waitTicks ) == pdTRUE )
	{	
//		if( length >= xStreamBufferBytesAvailable(USBD_VCOM_RxStreamBuffer) )
//			xEventGroupClearBits( CommuPortsRxEvents , CommuPort_USBD_VCOM );
		uint16_t rc_length = xStreamBufferReceive( USBD_VCOM_RxStreamBuffer , data , length , Rc_waitTicks );
//		if( xStreamBufferBytesAvailable(USBD_VCOM_RxStreamBuffer) > 0 )
//			xEventGroupSetBits( CommuPortsRxEvents , CommuPort_USBD_VCOM );
		
		//释放信号量
		xSemaphoreGive( USBD_VCOM_RxSemphr );
		return rc_length;
	}
	else
		return 0;
}

void init_drv_USB(void)
{
	//发送函数互斥锁
	USBD_VCOM_TxSemphr = xSemaphoreCreateRecursiveMutex();
	//接收函数互斥锁
	USBD_VCOM_RxSemphr = xSemaphoreCreateMutex();
	//发送缓冲区
	USBD_VCOM_TxStreamBuffer = xStreamBufferCreate( TxStreamBufferSize , 1 );
	//接收缓冲区
	USBD_VCOM_RxStreamBuffer = xStreamBufferCreate( RxStreamBufferSize , 1 );
		
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    while(1);
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    while(1);
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    while(1);
  }
	//USBD_CDC_SetRxBuffer(&hUsbDeviceFS, USBD_VCOM_RxBuffer);
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    while(1);
  }
  //HAL_PWREx_EnableUSBVoltageDetector();
	
	Port USBDVCOM;
	USBDVCOM.write = Write_USBD_VCOM;
	USBDVCOM.lock = Lock_USBD_VCOM;
	USBDVCOM.unlock = Unlock_USBD_VCOM;
	USBDVCOM.read = Read_USBD_VCOM;
	CommuPortRegister(USBDVCOM);
}


/*USB发送中断服务程序*/
	BaseType_t USBD_INT_HigherPriorityTaskWoken = pdFALSE;
	static volatile bool Tx_Complete = false;
	extern "C" void USBD_VCOM_TX_Complete_Callback()
	{
		Tx_Complete = true;
	}

	extern "C" void OTG_FS_IRQHandler(void)
	{
		extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
		HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
		
		BaseType_t HigherPriorityTaskWoken = USBD_INT_HigherPriorityTaskWoken;
		USBD_INT_HigherPriorityTaskWoken = pdFALSE;
		if( Tx_Complete )
		{
			uint16_t length = xStreamBufferReceiveFromISR( USBD_VCOM_TxStreamBuffer , USBD_VCOM_TxBuffer , TxBufferSize , &HigherPriorityTaskWoken );
			if( length > 0 )
				CDC_Transmit_FS( USBD_VCOM_TxBuffer , length );
			Tx_Complete = false;
		}
		portYIELD_FROM_ISR(HigherPriorityTaskWoken);
	}
/*USB发送中断服务程序*/
	
/*USB接收服务程序*/
	extern "C" void CDC_Receive_FS_Callback(uint8_t* pbuf, uint32_t Len)
	{
		if( Len > 0 )
			xStreamBufferSendFromISR( USBD_VCOM_RxStreamBuffer , pbuf , Len , &USBD_INT_HigherPriorityTaskWoken );
		//xEventGroupSetBitsFromISR( CommuPortsRxEvents , CommuPort_USBD_VCOM , &USBD_INT_HigherPriorityTaskWoken );
	}
/*USB接收服务程序*/

