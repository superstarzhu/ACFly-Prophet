#pragma once

#include "Modes.hpp"

class M11_TempCalib:public Mode_Base 
{
	private:
		
	public:
		M11_TempCalib();
		virtual ModeResult main_func( void* param1, uint32_t param2 );
};