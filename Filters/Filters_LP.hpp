#pragma once

#include "AC_Math.hpp"

inline bool BUT_IIR_calc_freq( double k[3] , double sample_freq, const double cutoff_freq , double cp )
{	
	if ( (cutoff_freq <= 0.0001) || (sample_freq <= 1.99 * cutoff_freq) ) {
			// no filtering
		return false;
	}
	double cos_PI_cp = cos( Pi * cp );
	
	double fr = sample_freq / cutoff_freq;
	double ohm = tan(Pi/fr);
	double ohm2 = ohm*ohm;
	double c = 1.0+2.0*cos_PI_cp*ohm + ohm2;
	double inv_c = 1.0 / c;
	k[0] = ohm2 * inv_c;
	k[1] = 2.0*(ohm2-1.0) * inv_c;
	k[2] = (1.0-2.0*cos_PI_cp*ohm+ohm2) * inv_c;
	
	return true;
}

/*一阶低通*/
	class Filter_LP_IIR_1
	{
		private:
			double k;
			double out;
		
		public:
			inline void set_cutoff_frequency( const double sample_freq, const double cutoff_freq )
			{
				if( sample_freq < 0.001f || cutoff_freq < 0.001f )
					return;
				this->k = 2 * Pi * cutoff_freq / sample_freq;
			}
			inline void set_value( const double value )
			{
				this->out = value;
			}
		
			inline Filter_LP_IIR_1()
			{
				this->k = 0;
				this->set_value( 0 );
			}		
			inline Filter_LP_IIR_1( double sample_freq , double cutoff_freq )
			{
				set_cutoff_frequency( sample_freq , cutoff_freq );
				this->set_value( 0 );
			}
			inline Filter_LP_IIR_1( double sample_freq , double cutoff_freq , double initial_offset ):Filter_LP_IIR_1( sample_freq , cutoff_freq )
			{
				this->set_value ( initial_offset );
			}
			
			inline double run( double new_data )
			{
				out += k * ( new_data - out );
				return out;
			}
	};
/*一阶低通*/

/*二阶ButterWorth低通*/
	class Filter_Butter2_LP
	{
		private:
			bool available;	
		
			double k_1[3];
		
			double in_1[2];
			double out_1[2];
		public:
			//滤波器可用状态
			inline bool is_available()
			{
				return this->available;
			}
			inline void set_inavailable()
			{
				this->available = false;
			}
			
			//设置滤波频率
			inline bool set_cutoff_frequency( double FS , double FC )
			{
				this->available = BUT_IIR_calc_freq( this->k_1 , FS , FC , 1.0 / 4 );
				return this->available;
			}
			inline void set_cutoff_frequency_from( const Filter_Butter2_LP& filter )
			{
				this->k_1[0] = filter.k_1[0];	this->k_1[1] = filter.k_1[1];	this->k_1[2] = filter.k_1[2];
				this->available = filter.available;		
			}
			
			//设置初始值
			inline void reset( double initial_value )
			{
				this->in_1[0] = this->in_1[1] = initial_value;
				this->out_1[0] = this->out_1[1] = initial_value;
			}
			
			//构造函数
			Filter_Butter2_LP()
			{
				this->available = false;
				this->reset(0);
			}
			Filter_Butter2_LP( double FS , double FC )
			{
				set_cutoff_frequency( FS , FC );
				this->reset(0);
			}
			
			//滤波
			inline double get_result(){return this->out_1[0];}
			inline double run( double newdata )
			{
				if( this->available )
				{
					double out_1_2 = this->out_1[1];	this->out_1[1] = this->out_1[0];
					this->out_1[0] = this->k_1[0]*( newdata + 2*this->in_1[0] + this->in_1[1] ) - this->k_1[1]*this->out_1[1] - this->k_1[2]*out_1_2;			
					this->in_1[1] = this->in_1[0];	this->in_1[0] = newdata;
				}
				else
					reset( newdata );
				return this->out_1[0];
			}
	};
/*二阶ButterWorth低通*/

/*四阶ButterWorth低通*/
	class Filter_Butter4_LP
	{
		private:
			bool available;	
		
			double k_1[3];
			double k_2[3];
		
			double in_1[2];
			double out_1[2];
			double in_2[2];
			double out_2[2];
		
		public:
			//滤波器可用状态
			inline bool is_available()
			{
				return this->available;
			}
			inline void set_inavailable()
			{
				this->available = false;
			}
			
			//设置滤波频率
			inline bool set_cutoff_frequency( double FS , double FC )
			{
				if( BUT_IIR_calc_freq( this->k_1 , FS , FC , 1.0 / 8 ) )
				{
					BUT_IIR_calc_freq( this->k_2 , FS , FC , 3.0 / 8 );
					this->available = true;
					return true;
				}
				else
				{
					this->available = false;
					return false;
				}
			}
			inline void set_cutoff_frequency_from( const Filter_Butter4_LP& filter )
			{
				this->k_1[0] = filter.k_1[0];	this->k_1[1] = filter.k_1[1];	this->k_1[2] = filter.k_1[2];
				this->k_2[0] = filter.k_2[0];	this->k_2[1] = filter.k_2[1];	this->k_2[2] = filter.k_2[2];
				this->available = filter.available;		
			}
			
			//设置初始值
			inline void reset( double initial_value )
			{
				this->in_1[0] = this->in_1[1] = initial_value;
				this->out_1[0] = this->out_1[1] = initial_value;
				
				this->in_2[0] = this->in_2[1] = initial_value;
				this->out_2[0] = this->out_2[1] = initial_value;
			}
			
			//构造函数
			Filter_Butter4_LP()
			{
				this->available = false;
				this->reset(0);
			}
			Filter_Butter4_LP( double FS , double FC )
			{
				set_cutoff_frequency( FS , FC );
				this->reset(0);
			}
			
			//滤波
			inline double get_result(){return this->out_2[0];}
			inline double run( double newdata )
			{
				if( this->available )
				{
					float out_1_2 = this->out_1[1];	this->out_1[1] = this->out_1[0];
					this->out_1[0] = this->k_1[0]*( newdata + 2*this->in_1[0] + this->in_1[1] ) - this->k_1[1]*this->out_1[1] - this->k_1[2]*out_1_2;			
					this->in_1[1] = this->in_1[0];	this->in_1[0] = newdata;
					
					newdata = this->out_1[0];
					float out_2_2 = this->out_2[1];	this->out_2[1] = this->out_2[0];
					this->out_2[0] = this->k_2[0]*( newdata + 2*this->in_2[0] + this->in_2[1] ) - this->k_2[1]*this->out_2[1] - this->k_2[2]*out_2_2;			
					this->in_2[1] = this->in_2[0];	this->in_2[0] = newdata;
				}
				else
					this->reset(newdata);
				return this->out_2[0];
			}
	};
/*四阶ButterWorth低通*/

/*八阶ButterWorth低通*/
	class Filter_Butter8_LP
	{
		private:
			bool available;	
		
			double k_1[3];
			double k_2[3];
			double k_3[3];
			double k_4[3];
		
			float in_1[2];
			float out_1[2];
			float in_2[2];
			float out_2[2];
			float in_3[2];
			float out_3[2];
			float in_4[2];
			float out_4[2];
		
		public:
			//滤波器可用状态
			inline bool is_available()
			{
				return this->available;
			}
			inline void set_inavailable()
			{
				this->available = false;
			}
		
			//设置滤波频率
			inline void set_cutoff_frequency_from( const Filter_Butter8_LP& filter )
			{
				this->k_1[0] = filter.k_1[0];	this->k_1[1] = filter.k_1[1];	this->k_1[2] = filter.k_1[2];
				this->k_2[0] = filter.k_2[0];	this->k_2[1] = filter.k_2[1];	this->k_2[2] = filter.k_2[2];
				this->k_3[0] = filter.k_3[0];	this->k_3[1] = filter.k_3[1];	this->k_3[2] = filter.k_3[2];
				this->k_4[0] = filter.k_4[0];	this->k_4[1] = filter.k_4[1];	this->k_4[2] = filter.k_4[2];
				this->available = filter.available;		
			}
			inline bool set_cutoff_frequency( double FS , double FC )
			{
				if( BUT_IIR_calc_freq( this->k_1 , FS , FC , 1.0f / 16 ) )
				{
					BUT_IIR_calc_freq( this->k_2 , FS , FC , 3.0f / 16 );
					BUT_IIR_calc_freq( this->k_3 , FS , FC , 5.0f / 16 );
					BUT_IIR_calc_freq( this->k_4 , FS , FC , 7.0f / 16 );
					this->available = true;
					return true;
				}
				else
				{
					this->available = false;
					return false;
				}
			}
			
			//设置初始值
			inline void reset( double initial_value )
			{
				this->in_1[0] = this->in_1[1] = initial_value;
				this->out_1[0] = this->out_1[1] = initial_value;
				
				this->in_2[0] = this->in_2[1] = initial_value;
				this->out_2[0] = this->out_2[1] = initial_value;
				
				this->in_3[0] = this->in_3[1] = initial_value;
				this->out_3[0] = this->out_3[1] = initial_value;
				
				this->in_4[0] = this->in_4[1] = initial_value;
				this->out_4[0] = this->out_4[1] = initial_value;
			}
			
			//构造函数
			Filter_Butter8_LP()
			{
				this->available = false;
				this->reset(0);
			}
			Filter_Butter8_LP( double FS , double FC )
			{
				set_cutoff_frequency( FS , FC );
				this->reset(0);
			}
			
			//滤波
			inline double get_result(){return this->out_4[0];}
			inline double run( double newdata )
			{
				if( this->available )
				{
					float out_1_2 = this->out_1[1];	this->out_1[1] = this->out_1[0];
					this->out_1[0] = this->k_1[0]*( newdata + 2*this->in_1[0] + this->in_1[1] ) - this->k_1[1]*this->out_1[1] - this->k_1[2]*out_1_2;			
					this->in_1[1] = this->in_1[0];	this->in_1[0] = newdata;
					
					newdata = this->out_1[0];
					float out_2_2 = this->out_2[1];	this->out_2[1] = this->out_2[0];
					this->out_2[0] = this->k_2[0]*( newdata + 2*this->in_2[0] + this->in_2[1] ) - this->k_2[1]*this->out_2[1] - this->k_2[2]*out_2_2;			
					this->in_2[1] = this->in_2[0];	this->in_2[0] = newdata;
					
					newdata = this->out_2[0];
					float out_3_2 = this->out_3[1];	this->out_3[1] = this->out_3[0];
					this->out_3[0] = this->k_3[0]*( newdata + 2*this->in_3[0] + this->in_3[1] ) - this->k_3[1]*this->out_3[1] - this->k_3[2]*out_3_2;			
					this->in_3[1] = this->in_3[0];	this->in_3[0] = newdata;
					
					newdata = this->out_3[0];
					float out_4_2 = this->out_4[1];	this->out_4[1] = this->out_4[0];
					this->out_4[0] = this->k_4[0]*( newdata + 2*this->in_4[0] + this->in_4[1] ) - this->k_4[1]*this->out_4[1] - this->k_4[2]*out_4_2;			
					this->in_4[1] = this->in_4[0];	this->in_4[0] = newdata;
				}
				else
					reset( newdata );
				return this->out_4[0];
			}
	};
