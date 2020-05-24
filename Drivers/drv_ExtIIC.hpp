#pragma once

/*上锁保证通信连续性
	上锁之后必须解锁
	Sync_waitTime：超时时间
*/
bool Lock_ExtIIC( double Sync_waitTime = -1 );
void Unlock_ExtIIC();

/*7位地址发送数据
	addr：7位从器件地址
	datas：要发送的数据指针
	length：数据长度
	Sync_waitTime：超时时间
*/
bool ExtIIC_SendAddr7( uint8_t addr, const uint8_t* datas, uint16_t length, double Sync_waitTime = -1 );

/*7位地址发送并接收数据
	addr：7位从器件地址
	datas：要发送的数据指针
	length：数据长度
	Sync_waitTime：超时时间
*/
bool ExtIIC_SendReceiveAddr7( uint8_t addr, const uint8_t* tx_datas, uint16_t tx_length, const uint8_t* rx_datas, uint16_t rx_length, double Sync_waitTime = -1 );

void init_drv_ExtIIC();