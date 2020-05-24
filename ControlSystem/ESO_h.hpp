#pragma once

//高度无参ESO
//朱文杰 20180102
//请勿用于商业用途
//！！抄袭必究！！

#include <stdbool.h>
#include "RingQueue.hpp"
#include "AC_Math.hpp"

//观测量延时
#define ESO_h_his_length 16

class ESO_h
{
	private:
		double beta;
		
		double T;
		double invT;
		double z_inertia;
		double z1;
		double z2;
		double u;
		
		RingQueue<double> his_zin;

		double h;

		bool err_sign;
		double err_continues_time;
	
	public:		
		inline void init( double T, double beta )
		{			
			this->T = T;
			this->invT = 1.0f / T;
			this->beta = beta;
			this->z1 = this->u = 0;
			this->z2 =1.0f;			
		}
		ESO_h():his_zin(ESO_h_his_length)
		{
			for( uint8_t i = 0; i < ESO_h_his_length; ++i )
				his_zin.push(0);
		}

		inline void update_u( double u )
		{
			this->u = u;
			this->z_inertia += this->h * this->invT * ( this->u - this->z_inertia );
		}

		inline double run( double acc, double h )
		{
			if( this->z2 < 1.0f )
				this->z2 = 1.0f;
			double b = GravityAcc / this->z2;
			double his_zinertial = *this->his_zin.get_current();
			if( his_zinertial > 0.1f )
			{
				double actual_z2 = his_zinertial / ( acc + GravityAcc ) * GravityAcc;
				double err = actual_z2 - this->z2;
				if( (err > 0) ^ this->err_sign )
				{
					this->err_continues_time = 0;
					this->err_sign = err > 0;
				}
				else
					this->err_continues_time += h;
				
				if( err > 10 )
					err = 10;
				else if( err < -10 )
					err = -10;
				double beta_scale = this->beta * ( 1 + 500*this->err_continues_time*this->err_continues_time*this->err_continues_time );
				if( beta_scale > 0.2f )
					beta_scale = 0.2f;
				this->z2 += beta_scale * err;
			}
			if( this->z2 < 1.0f )
				this->z2 = 1.0f;
			this->his_zin.push(this->z_inertia);
			
			this->h = h;
			return this->z2;
		}
};

