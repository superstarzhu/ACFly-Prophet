#pragma once

#include "Modes.hpp"

class M30_AttCtrl:public Mode_Base 
{
	private:
		
	public:
		M30_AttCtrl();
		virtual ModeResult main_func( void* param1, uint32_t param2 );
};