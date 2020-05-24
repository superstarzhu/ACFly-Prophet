#include "mavlink.h"
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t m_mavlink_TxMutex[MAVLINK_COMM_NUM_BUFFERS] = {0};

mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];

mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];

extern "C"
{
	void mavlink_init_chan( uint8_t chan )
	{
		if( m_mavlink_TxMutex[chan] == 0 )
			m_mavlink_TxMutex[chan] = xSemaphoreCreateMutex();
	}
	bool mavlink_lock_chan( uint8_t chan, double TIMEOUT )
	{
		if( m_mavlink_TxMutex[chan] == 0 )
			return false;
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( m_mavlink_TxMutex[chan], TIMEOUT_Ticks ) )
			return true;
		return false;
	}
	void mavlink_unlock_chan( uint8_t chan )
	{
		xSemaphoreGive( m_mavlink_TxMutex[chan] );
	}
}