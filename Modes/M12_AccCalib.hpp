#pragma once

#include "Modes.hpp"

class M12_AccCalib:public Mode_Base 
{
	private:
		
	public:
		M12_AccCalib();
		virtual ModeResult main_func( void* param1, uint32_t param2 );
};