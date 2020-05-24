#pragma once
#include "mavlink.h"

extern bool (*const Mavlink_Send_Funcs[])( uint8_t port , mavlink_message_t* msg_sd );
extern const uint16_t Mavlink_Send_Funcs_Count;