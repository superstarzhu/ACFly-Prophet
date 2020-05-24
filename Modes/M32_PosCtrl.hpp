#pragma once

#include "Modes.hpp"

class M32_PosCtrl:public Mode_Base 
{
	private:
		
	public:
		M32_PosCtrl();
		virtual ModeResult main_func( void* param1, uint32_t param2 );
};