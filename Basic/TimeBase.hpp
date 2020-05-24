#pragma once

#include "stm32h743xx.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "TimeBase.h"

	namespace __global_time
	{
		extern const uint32_t timer_load;
		
		extern volatile uint32_t current_time_part_1;
		extern const double TIM2sec_scale;
		extern const double TIM2sec_part1_scale;
	}

	class TIME
	{
		public:
			uint64_t t;
		
			TIME(){t=0;};
			TIME(bool valid)
			{
				if( valid) t = now().t;
				else this->set_invalid();
			}
		
			inline TIME operator =( const TIME& b )
			{
				t = b.t;
				return *this;
			}
			
			inline TIME operator +=( const TIME& b )
			{
				t += b.t;
				return *this;
			}
			inline TIME operator -=( const TIME& b )
			{
				t -= b.t;
				return *this;
			}
			inline TIME operator +=( const double& b )
			{
				t += b * TIMEBASECLK;
				return *this;
			}
			inline TIME operator -=( const double& b )
			{
				t -= b * TIMEBASECLK;
				return *this;
			}
			
			inline bool operator ==(const TIME &b) const
			{
					return t == b.t;
			}
			inline bool operator !=(const TIME &b) const
			{
					return t != b.t;
			}
			
			inline void set_invalid(){ this->t = 0; }
			inline bool is_valid(){ return this->t != 0; }
			
			inline static TIME now()
			{
				TIME t_now;
				
				uint32_t part1;
				uint32_t part2;
				do
				{
					part1 = __global_time::current_time_part_1;
					part2 = TIM5->CNT;
				}
				while( part1 != __global_time::current_time_part_1 );
				t_now.t = (uint64_t)part1*(uint64_t)__global_time::timer_load + (uint64_t)part2;
				return t_now;
			}
			inline static double get_System_Run_Time()
			{
				TIME current_time = TIME::now();
				union long_int_union
				{
					uint64_t long_data;
					unsigned int int_data[2];
				};
				uint64_t time_diff = current_time.t;
				long_int_union* convert = (long_int_union*)&time_diff;
				return __global_time::TIM2sec_part1_scale * convert->int_data[1] + __global_time::TIM2sec_scale * convert->int_data[0];
			}
			
			inline double get_pass_time_fromStartUp() const
			{
				union long_int_union
				{
					uint64_t long_data;
					unsigned int int_data[2];
				};
				uint64_t time_diff = this->t;
				long_int_union* convert = (long_int_union*)&time_diff;
				return __global_time::TIM2sec_part1_scale * convert->int_data[1] + __global_time::TIM2sec_scale * convert->int_data[0];
			}
		
			inline double get_pass_time() const
			{
				TIME current_time = TIME::now();
				union long_int_union
				{
					uint64_t long_data;
					unsigned int int_data[2];
				};
				if( current_time.t > this->t )
				{
					uint64_t time_diff = current_time.t - this->t;
					long_int_union* convert = (long_int_union*)&time_diff;
					return __global_time::TIM2sec_scale * time_diff;
				}
				else
				{
					uint64_t time_diff = this->t - current_time.t;
					long_int_union* convert = (long_int_union*)&time_diff;
					return -__global_time::TIM2sec_scale * time_diff;
				}
			}
			inline double get_pass_time_st()
			{
				TIME current_time = now();
				union long_int_union
				{
					uint64_t long_data;
					unsigned int int_data[2];
				};
				if( current_time.t > this->t )
				{
					uint64_t time_diff = current_time.t - this->t;
					long_int_union* convert = (long_int_union*)&time_diff;
					this->t = current_time.t;
					return __global_time::TIM2sec_scale * time_diff;
				}
				else
				{
					uint64_t time_diff = this->t - current_time.t;
					long_int_union* convert = (long_int_union*)&time_diff;
					this->t = current_time.t;
					return -__global_time::TIM2sec_scale * time_diff;
				}
			}
			
			inline static double get_time_difference( const TIME& time_new , const TIME& time_old )
			{
				if( time_new.t > time_old.t )
					return __global_time::TIM2sec_scale * ( time_new.t - time_old.t );
				else
					return -__global_time::TIM2sec_scale * ( time_old.t - time_new.t );
			}
	};
	
	/*
		阻塞延时函数
		t:秒单位延时时间
	*/
	inline void delay(double t)
	{
		TIME start_time = TIME::now();
		while( start_time.get_pass_time() < t );
	}
	
	void init_TimeBase();