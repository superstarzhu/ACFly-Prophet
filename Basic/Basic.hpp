#pragma once

#include "TimeBase.hpp"
#include "stm32h743xx.h"

#include <string.h>
#include "Basic.h"

/*
	可比较定长名称类
	用于名称存储，索引
*/
class SName
{
	private:
		char name[16];
	
	public:
		SName(){ memset( this->name , 0 , 16 ); }
		SName( const char* name )
		{
			uint8_t i;
			for( i = 0 ; i < 16 ; ++i )
			{
				this->name[i] = name[i];
				if( name[i] == 0 )
					break;				
			}
			memset( &this->name[i] , 0 , 16-i );
		}
		
		//获取字符串（17字节长度）
		void get_CharStr( char* str )
		{
			str[16] = 0;
			memcpy( str , name , 16 );
		}
		
		inline bool operator ==(const SName &n2) const
		{
			for( uint8_t i = 0; i < 16; ++i )
			{
				if( this->name[i] != n2.name[i] )
					return false;
				if( this->name[i] == 0 )
					return true;
			}
			return true;
		}
		
		inline bool operator !=(const SName &n2) const
		{
			for( uint8_t i = 0; i < 16; ++i )
			{
				if( this->name[i] != n2.name[i] )
					return true;
				if( this->name[i] == 0 )
					return false;
			}
			return false;
		}
		
		//比较运算符用于排序
		bool operator<(const SName& name) const
		{
			for( uint8_t i = 0; i < 16; ++i )
			{
				if( this->name[i] < name.name[i] )
					return true;
				else if( this->name[i] > name.name[i] )
					return false;
				if( this->name[i] == 0 )
					return false;
			}
			return false;
		}
		
		//名称拼接
		SName operator+(const SName &n)
    {
			uint8_t i;
			SName result;
			for( i = 0 ; i < 16 ; ++i )
			{
				if( name[i] == 0 )
					break;		
				result.name[i] = this->name[i];		
			}
			uint8_t ni = i;
      for(  ; i < 16 ; ++i )
			{
				result.name[i] = n.name[i-ni];
				if( n.name[i-ni] == 0 )
					break;	
			}
			return result;
    }
};

//初始化完成指示
//初始化完成不能进行初始化操作
bool getInitializationCompleted();
void setInitializationCompleted();
void LockInitializationStatus();
void UnLockInitializationStatus();

/*
	配置寄存器
	reg:寄存器指针
	value:要填入指定位中的值
	offset:要填入的位的偏移
	value_length:要填入的位数
*/
inline void set_register( volatile unsigned int& reg , const unsigned char value , const unsigned char offset , const unsigned char value_length )
{
	unsigned char offset_end_bit = offset + value_length;
	for( unsigned char i = offset ; i < offset_end_bit ; ++ i )
		reg &= ~(1<<i);
	
	reg |= ( value << offset );
}

void init_Basic();