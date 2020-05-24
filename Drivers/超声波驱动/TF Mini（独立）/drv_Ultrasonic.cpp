#include "Basic.h"
#include "drv_Uart2.h"

#include "Quaternion.h"
#include "MeasurementSystem.h"
#include "drv_SDI.h"
#include "Commulink.h"
#include "STS.h"
#include "Sensors_Backend.h"
#include "RingBuf.h"

#include "TM4C123GH6PM.h"
#include "uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_gpio.h"
#include "Timer.h"
#include "udma.h"



/*接收缓冲区*/
#define RX_BUFFER_SIZE 24
static uint8_t R7_rx_buffer[RX_BUFFER_SIZE];
static uint8_t R7_Rx_RingBuf;
static float HIGH_EIGHT;
static float LOW_EIGHT;
static float ks109_HIGH;
static void UART7_Handler(void);
static void UART7_receive(void);
static void ks109_send(unsigned int Task_ID);
void drv_ks109(void)
{
   
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	 GPIOPinConfigure(GPIO_PE0_U7RX);
	 GPIOPinConfigure(GPIO_PE1_U7TX);
	 
	 GPIOPinTypeUART(GPIOE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	 UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet() , 115200,			
	                   (UART_CONFIG_WLEN_8 
										| UART_CONFIG_STOP_ONE 
										|	UART_CONFIG_PAR_NONE));	
	 UARTFIFOEnable( UART7_BASE );	
	
	//配置串口接收中断
	UARTIntEnable( UART7_BASE , UART_INT_RX | UART_INT_RT );
	UARTIntRegister( UART7_BASE , UART7_Handler );	 
	
	UARTFIFOEnable( UART7_BASE );
	UARTIntEnable(UART7_BASE,UART_INT_RX | UART_INT_RT);//使能UART0发送接收中断		
  UARTIntRegister(UART7_BASE,UART7_Handler);//UART中断地址注册	
	IntPrioritySet(INT_UART7, INT_PRIO_7);
	//STS_Add_Task( STS_Task_Trigger_Mode_RoughTime , 1.0f/10 , 0 , send );
  PositionSensorRegister( default_ultrasonic_sensor_index , \
													Position_Sensor_Type_RangePositioning , \
													Position_Sensor_DataType_s_z , \
													Position_Sensor_frame_ENU , \
													0.05f , \
													false );
	
 

}

uint16_t hign_z;
static uint8_t hbyte[9];

static void UART7_Handler()
{
	int i = 0;
	UARTIntClear( UART7_BASE , UART_INT_OE | UART_INT_RT );
	UARTRxErrorClear( UART7_BASE );
	while( ( UART7->FR & (1<<4) ) == false	)
	{
		//接收
		
    uint8_t rdata = UART7->DR & 0xff;
		static unsigned char rc_counter = 0;
		static unsigned char receive = 0;
		
		static uint16_t check_sum;
		static signed char sum = 0;
		
	  if( rc_counter < 2 )
		{
			//接收包头
			if( rdata != 0x59)
			{
				rc_counter = 0;
			}else
			{
				hbyte[rc_counter] = rdata;
				++rc_counter;
			}
		}else if( rc_counter < 9 )
		{
			hbyte[rc_counter] = rdata;
			++rc_counter;
		}else if(rc_counter == 9)
		{
			check_sum = hbyte[0]+hbyte[1]+hbyte[2]+hbyte[3]+hbyte[4]+hbyte[5]+hbyte[6]+hbyte[7];
			if((check_sum&0xFF) != hbyte[8])
			{
				rc_counter = 0;
			}
			else{
				++rc_counter;
			}
		}else
		{
			vector3_float position;
			position.z = ((float)(signed short)(((((unsigned short)hbyte[3])<<8)|(uint8_t)hbyte[2])));
				hign_z = position.z;
			if( position.z > 30 && position.z < 500 )
			{
				PositionSensorUpdatePosition( default_ultrasonic_sensor_index , position , true , -1 );
			}
			else
			{
				PositionSensorSetInavailable( default_ultrasonic_sensor_index );
			}
			rc_counter = 0;
		}
		
			
				
		}
		
}
	

	uint16_t get_position_z()
	{
		return hign_z;
	}