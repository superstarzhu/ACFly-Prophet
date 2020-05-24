#pragma once

#include "Modes.hpp"

class M10_RCCalib:public Mode_Base 
{
	private:
		
	public:
		M10_RCCalib();
		virtual ModeResult main_func( void* param1, uint32_t param2 );
};