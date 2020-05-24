#pragma once

#include "Basic.hpp"

struct ModeMsg
{
	uint8_t cmd_type;
	uint32_t cmd;
	float params[8];
};
bool ModeReceiveMsg( ModeMsg* msg, double TIMEOUT );
bool SendMsgToMode( ModeMsg msg, double TIMEOUT );

class Mode_Base;
void ModeRegister( Mode_Base* mode, uint8_t id );

enum ModeResult
{
	MR_OK = 0 ,
	MR_Err ,
};

class Mode_Base
{
	private:
		
	public:
		SName name;
		Mode_Base( SName name, uint8_t mode_id )
		{
			this->name = name;
			ModeRegister( this, mode_id );
		}		
		//模式主函数
		virtual ModeResult main_func( void* param1, uint32_t param2 ) = 0;
};



void init_Modes();