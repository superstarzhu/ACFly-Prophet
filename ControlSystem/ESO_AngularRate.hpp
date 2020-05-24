#pragma once

//角速度带模型ESO
//朱文杰 20180102
//请勿用于商业用途
//！！抄袭必究！！

#include <stdbool.h>
#include "AC_Math.hpp"

//观测量延时
#define ESO_AngularRate_his_length 8

class ESO_AngularRate
{
	private:
		double invT;
		double z_inertia;
		double z1;
		double z2;
		double u;
		double last_err;
		double his_z1[ ESO_AngularRate_his_length ];

		double h;

		bool err_sign;
		double err_continues_time;
		
			
	
	public:	
		double beta1;
		double beta2;
		
		double ceta1;
		double ceta2;
	
		double T;
		double b;
		
	
		inline void init( double T , double b , double beta1 , double beta2 , double ceta1 , double ceta2 )
		{
			this->beta1 = beta1;
			this->beta2 = beta2;
			
			this->ceta1 = ceta1;
			this->ceta2 = ceta2;
			
			this->z1 = this->z2 = this->z_inertia = 0;
			this->err_continues_time = 0;	
			
			this->T = T;	this->invT = 1.0f / T;
			this->b = b;
			for( unsigned char i = 0 ; i < ESO_AngularRate_his_length ; ++i )
				this->his_z1[i] = 0;
		}
		
		inline void update_u( double u )
		{
			this->u = u;
			this->z_inertia += this->h * this->invT * ( this->b*this->u - this->z_inertia );
			this->z1 += this->h * ( this->z_inertia + this->z2 );
		}
		
		inline double run( double v, double h )
		{
			double err = v - this->his_z1[0];
			double z2_err = err - this->last_err;
			
			//更新误差持续时间
			if( (z2_err > 0) ^ this->err_sign )
			{
				this->err_continues_time = 0;
				this->err_sign = z2_err > 0;
			}
			else
				this->err_continues_time += h;
			
			double max_beta1_scale = 0.9f / this->beta1;
			double err_continues_time3 = this->err_continues_time*this->err_continues_time*this->err_continues_time;
			double beta1_scale = 1 + 0*this->ceta1 * err_continues_time3;	
			double beta2_scale = 1 + this->ceta2 * err_continues_time3;		
			if( beta1_scale > 15 )
				beta1_scale = 15;
			if( beta2_scale > 5 )
				beta2_scale = 5;
			if( beta1_scale > max_beta1_scale )
				beta1_scale = max_beta1_scale;
			
			//修正
			double z1_correction = beta1_scale*this->beta1*err;
			double z2_correction = beta2_scale*this->beta2*z2_err;
			double max_z2_correction = z1_correction / ( ESO_AngularRate_his_length*h );
			z1_correction -= z2_correction * ESO_AngularRate_his_length*h;
			double filter_dt = h;
			for( unsigned char k = 0 ; k < ESO_AngularRate_his_length - 1 ; ++k )
			{
				this->his_z1[ k ] = this->his_z1[ k + 1 ] + z1_correction + filter_dt*z2_correction;
				filter_dt += h;
			}
			this->z2 += z2_correction;
			this->z1 += z1_correction + filter_dt*z2_correction;
			this->his_z1[ ESO_AngularRate_his_length - 1 ] = this->z1;
			this->last_err = err - z1_correction;
			
			this->h = h;
			return this->z2;
		}
		
		inline double get_EsAngularRate() const
		{
			return this->z1;
		}
		inline double get_EsDisturbance() const
		{
			return this->z2;
		}
		inline double get_EsAngularAcceleration() const
		{
			return this->z2 + this->z_inertia;
		}
		inline double get_EsMainPower() const
		{
			return this->z_inertia;
		}
};