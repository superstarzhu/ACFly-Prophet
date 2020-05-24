#pragma once

#include <stdint.h>

/*发送上锁
	上锁保证数据连续性
	上锁之后必须解锁
*/
bool Lock_USBD_VCOM( double Sync_waitTime = -1 );
void Unlock_USBD_VCOM();

/*USBD虚拟串口发送函数（将要发送的数据压入缓冲区）
	data:要发送的数据指针
	length:要发送的数据长度（字节）
	Sync_waitTime:线程同步最大等待时间（s）
	Send_waitTime:等待缓冲区有空间的最大等待时间（s）
	返回值：实际发送的字节数
	解释：如果在指定等待时间内接收不到足够数据（发送缓冲区位置不足）
				将只发送前半段数据（只将前半段数据压入缓冲区）
*/
uint16_t Write_USBD_VCOM( const uint8_t *data, uint16_t length, double Send_waitTime = 0, double Sync_waitTime = -1 );

/*USBD虚拟串口接收函数（将要发送的数据压入缓冲区）
	data:接收数据指针
	length:要接收的数据长度（字节）
	Sync_waitTime:线程同步最大等待时间（s）
	Rc_waitTime:等待数据的最大等待时间（s）
	返回值：实际接收到的字节数
	解释:如果在指定等待时间内接收不到足够数据（接收缓冲区里没这么多数据）
				就接收尽量多的数据
*/
uint16_t Read_USBD_VCOM( uint8_t *data, uint16_t length, double Rc_waitTime = 0, double Sync_waitTime = -1 );

void init_drv_USB(void);