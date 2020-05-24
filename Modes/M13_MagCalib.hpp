#pragma once

#include "Modes.hpp"

class M13_MagCalib:public Mode_Base 
{
	private:
		
	public:
		M13_MagCalib();
		virtual ModeResult main_func( void* param1, uint32_t param2 );
};