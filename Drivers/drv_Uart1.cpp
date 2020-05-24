#include "Basic.hpp"
#include "drv_Uart1.hpp"
#include "Commulink.hpp"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "stream_buffer.h"

//发送互斥锁
static SemaphoreHandle_t TxSemphr;
//USB发送消息缓冲区
#define TxStreamBufferSize 1024
#define TxBufferSize 64
Static_AXIDMABuf uint8_t TxBuffer[TxBufferSize];
static StreamBufferHandle_t TxStreamBuffer;
static bool TxCompleted = true;
//发送完成标志
static EventGroupHandle_t events = xEventGroupCreate();

//接收互斥锁
static SemaphoreHandle_t RxSemphr;
//接收缓冲区
#define RxStreamBufferSize 1024
static StreamBufferHandle_t RxStreamBuffer;

static inline void StartSend( const uint8_t* buf, uint16_t len )
{
	//清空发送完成标志
	TxCompleted = false;
	xEventGroupClearBits( events, (1<<0) );	
	//关闭串口DMA发送
	USART1->CR3 &= ~(1<<7);
	//清空DMA状态
	DMA1->LIFCR = (1<<27) | (1<<26)  | (1<<25)  | (1<<24)  | (1<<22);
	//设置DMA存储器地址
	DMA1_Stream3->M0AR = (uint32_t)buf;
	//设置DMA传输数量
	DMA1_Stream3->NDTR = len;
	//使能DMA
	DMA1_Stream3->CR |= (1<<0);	
	//清空Uart状态
	USART1->ICR = (1<<6);	
	//打开串口DMA发送
	USART1->CR3 |= (1<<7);
}

static inline void setBaudRate( uint32_t baud )
{
	USART1->CR1 &= ~( (1<<6) | (1<<3) );
	USART1->CR1 &= ~(1<<0);
	USART1->BRR = USART16CLK / baud;
	USART1->CR1 |= (1<<6) | (1<<3) | (1<<0);
}

void init_drv_Uart1()
{
	/*端口初始化*/
		TxSemphr = xSemaphoreCreateRecursiveMutex();
		RxSemphr = xSemaphoreCreateMutex();
		//发送缓冲区
		TxStreamBuffer = xStreamBufferCreate( TxStreamBufferSize , 1 );
		//接收缓冲区
		RxStreamBuffer = xStreamBufferCreate( RxStreamBufferSize , 1 );
	/*端口初始化*/
	
	/*GPIO初始化*/
		//PA9(USART1 TX) PA10(USART1 RX)
	
		//打开GPIOA时钟
		RCC->AHB4ENR|=(1<<0);
		os_delay(1e-2);
	
		//复用功能 TX推挽 RX开漏上拉
		GPIOA->OTYPER |= (1<<10);
		set_register( GPIOA->PUPDR, 0b01, 10*2 , 2 );
		set_register( GPIOA->AFR[1], 7, 9*4-32, 4 );
		set_register( GPIOA->AFR[1], 7, 10*4-32, 4 );
		set_register( GPIOA->MODER, 0b10, 9*2, 2);
		set_register( GPIOA->MODER, 0b10, 10*2, 2);		
	/*GPIO初始化*/
	
	/*Usart1初始化*/
		//打开Usart1时钟
		RCC->APB2ENR|=(1<<4);
		os_delay(1e-2);
	
		USART1->CR1 = (1<<29) | (1<<26) | (1<<6) | (1<<3) | (1<<2);
		setBaudRate(115200);
		USART1->RTOR = 10;
		USART1->CR2 = (1<<23);
		USART1->CR3 = (1<<28) | (0b011<<25);		
		USART1->CR1 |= (1<<0);    //USART使能	
		NVIC_SetPriority(USART1_IRQn,5);
		NVIC_EnableIRQ(USART1_IRQn);
		xEventGroupSetBits( events, (1<<0) );
	/*Usart1初始化*/
	
	/*DMA初始化*/
		//打开DMA1时钟
		RCC->AHB1ENR |= (1<<0);
		delay(1e-5);
		
		//DMA1_Stream3 Uart1 TX
		DMA1_Stream3->PAR = (uint32_t)&USART1->TDR;
		DMAMUX1_Channel3->CCR = (42<<0);
		DMA1_Stream3->CR = (1<<20) | (0<<16) | (0<<13) | (1<<10) | (0<<9) | (0b01<<6);
		DMA1_Stream3->FCR = (1<<2) | (3<<0);
	/*DMA初始化*/
	
	//端口注册
	Port UartPort;
	UartPort.write = Write_Uart1;
	UartPort.lock = Lock_Uart1;
	UartPort.unlock = Unlock_Uart1;
	UartPort.read = Read_Uart1;
	CommuPortRegister(UartPort);
}

static void TxTCB( void *pvParameter1, uint32_t ulParameter2 )
{
	uint16_t length = xStreamBufferReceive( TxStreamBuffer, TxBuffer, TxBufferSize, 0 );
	if( length > 0 )
		StartSend( TxBuffer, length );
	else
	{
		TxCompleted = true;
		xEventGroupSetBits( events, (1<<0) );
	}
}
extern "C" void USART1_IRQHandler()
{
	bool err = false;
	if( USART1->ISR & (1<<3) )
	{	//接收fifo溢出
		err = true;
		USART1->ICR = 1<<3;
	}
	
	BaseType_t HigherPriorityTaskWoken = pdFALSE;
	if( (USART1->ISR & (1<<11)) || (USART1->ISR & (1<<26)) )
	{	//接收中断
		USART1->ICR = 1<<11;
		
		uint8_t buf[18];
		uint8_t len = 0;
		while( ( USART1->ISR & (1<<5) ) != 0 )
			buf[len++] = USART1->RDR;
		if( len > 0 )
			xStreamBufferSendFromISR( RxStreamBuffer , buf , len , &HigherPriorityTaskWoken );
	}
	if( (USART1->ISR & (1<<6)) )
	{	//发送完成中断
		USART1->ICR = (1<<6);
		xTimerPendFunctionCallFromISR( TxTCB, 0, 0, &HigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

/*串口接收函数（将要发送的数据压入缓冲区）
	data:接收数据指针
	length:要接收的数据长度（字节）
	Sync_waitTime:线程同步最大等待时间（s）
	Rc_waitTime:等待数据的最大等待时间（s）
	返回值：实际接收到的字节数
	解释:如果在指定等待时间内接收不到足够数据（接收缓冲区里没这么多数据）
				就接收尽量多的数据
*/
uint16_t Read_Uart1( uint8_t *data, uint16_t length, double Rc_waitTime, double Sync_waitTime )
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
	if( xSemaphoreTake( RxSemphr , Sync_waitTicks ) == pdTRUE )
	{	
		uint16_t rc_length = xStreamBufferReceive( RxStreamBuffer , data , length , Rc_waitTicks );
		
		//释放信号量
		xSemaphoreGive( RxSemphr );
		return rc_length;
	}
	else
		return 0;
}

/*串口发送函数（将要发送的数据压入缓冲区）
	data:要发送的数据指针
	length:要发送的数据长度（字节）
	Sync_waitTime:线程同步最大等待时间（s）
	Send_waitTime:等待缓冲区有空间的最大等待时间（s）
	返回值：实际发送的字节数
	解释：如果在指定等待时间内接收不到足够数据（发送缓冲区位置不足）
				将只发送前半段数据（只将前半段数据压入缓冲区）
*/
uint16_t Write_Uart1( const uint8_t *data, uint16_t length, double Send_waitTime, double Sync_waitTime )
{
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
	if( xSemaphoreTakeRecursive( TxSemphr , Sync_waitTicks ) == pdTRUE )
	{	
		uint16_t data_sent = xStreamBufferSend( TxStreamBuffer, data, length, Send_waitTicks );
		if( TxCompleted )
		{
			//USB空闲可发送
			uint16_t sd_length;
			sd_length = xStreamBufferReceive( TxStreamBuffer, TxBuffer, TxBufferSize, 0 );
			if( sd_length > 0 )
				StartSend( TxBuffer, sd_length );			
		}
		//释放信号量
		xSemaphoreGiveRecursive( TxSemphr );
		return data_sent;
	}
	else
		return 0;
}

/*发送上锁
	上锁保证数据连续性
	上锁之后必须解锁
*/
bool Lock_Uart1( double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTicks > 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	if( xSemaphoreTakeRecursive( TxSemphr , Sync_waitTicks ) == pdTRUE )
		return true;
	return false;
}
void Unlock_Uart1()
{
	xSemaphoreGiveRecursive( TxSemphr );
}

/*清空接收缓冲区
	Sync_waitTime:线程同步最大等待时间（s）
*/
bool ResetRx_Uart1( double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTime >= 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	//获取信号量
	if( xSemaphoreTake( RxSemphr , Sync_waitTicks ) == pdTRUE )
	{
		xStreamBufferReset(RxStreamBuffer);
		xSemaphoreGive(RxSemphr);
		return true;
	}
	return false;
}

/*等待发送完成
	waitTime:最大等待时间（s）
*/
bool WaitSent_Uart1( double waitTime )
{
	uint32_t waitTicks;
	if( waitTicks >= 0 )
		waitTicks = waitTime*configTICK_RATE_HZ;
	else
		waitTicks = portMAX_DELAY;
	EventBits_t rtbits = xEventGroupWaitBits( events, (1<<0), pdFALSE, pdTRUE, waitTicks );
	if( rtbits == (1<<0) )
		return true;
	return false;
}

/*更改波特率
	baud_rate:波特率
	Send_waitTime:等待发送完成的最大等待时间（s）
	Sync_waitTime:线程同步最大等待时间（s）
*/
bool SetBaudRate_Uart1( uint32_t baud_rate, double Send_waitTime, double Sync_waitTime )
{
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
	if( xSemaphoreTakeRecursive( TxSemphr , Sync_waitTicks ) == pdTRUE )
	{	
		EventBits_t rtbits = xEventGroupWaitBits( events, (1<<0), pdFALSE, pdTRUE, Send_waitTicks );
		bool result = false;
		if( rtbits == (1<<0) )
		{
			setBaudRate(baud_rate);
			result = true;
		}
		xSemaphoreGiveRecursive( TxSemphr );
		return result;
	}
	return false;
}
