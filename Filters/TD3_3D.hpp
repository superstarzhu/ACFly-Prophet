#pragma once

#include "AC_Math.hpp"
#include "vector3.hpp"
#include "vector2.hpp"
#include "smooth_kp.hpp"

class TD3_3DSL
{
	private:
		unsigned char tracking_mode;	
	
	public:
		vector3<double> x1;
		vector3<double> x2;
		vector3<double> x3;
		vector3<double> T4;
	
		double P1;
		double P2;
		double P3;
		
		double r2, r3, r4;
		
		inline unsigned char get_tracking_mode(){ return tracking_mode; }
		inline vector3<double> get_x1(){ return x1; }
		inline vector3<double> get_x2(){ return x2; }
		inline vector3<double> get_x3(){ return x3; }
	
		inline void reset()
		{
			this->x1.zero();
			this->x2.zero();
			this->x3.zero();
			tracking_mode = 0;
		}
		
		inline TD3_3DSL()
		{
			this->r2 = this->r3 = this->r4 = 1e12;			
			this->reset();
		}
		inline TD3_3DSL( double P1, double P2, double P3, double P4 )
		{
			this->P1 = P1;
			this->P2 = P2;
			this->P3 = P3;
			this->r2 = this->r3 = this->r4 = 1e12;			
			
			this->reset();
		}
		
		inline vector3<double> track3( vector3<double> expect , double h )
		{
			this->tracking_mode = 3;
	
			double e_1_n;
			double e_1;
			double e_2_n;
			double e_2;
			
			vector3<double> e1 = expect - this->x1;
			vector3<double> e1_1 = -this->x2;
			vector3<double> e1_2 = -this->x3;
			double e1_length = safe_sqrt(e1.get_square());
			
			e_1_n = e1.x*e1_1.x + e1.y*e1_1.y + e1.z*e1_1.z;
			if( !is_zero(e1_length) )
				e_1 = e_1_n / e1_length;
			else
				e_1 = 0;
			e_2_n = ( e1.x*e1_2.x + e1.y*e1_2.y + e1.z*e1_2.z + e1_1.x*e1_1.x + e1_1.y*e1_1.y + e1_1.z*e1_1.z )*e1_length - e_1*e_1_n;
			if( !is_zero(e1_length*e1_length) )
				e_2 = e_2_n / (e1_length*e1_length);
			else
				e_2 = 0;
			smooth_kp_d2 d1 = smooth_kp_2( e1_length, e_1, e_2 , this->P1, this->r2 );
			vector3<double> T2;
			vector3<double> T2_1;
			vector3<double> T2_2;
			if( !is_zero(e1_length*e1_length*e1_length) )
			{
				vector3<double> n = e1 * (1.0/e1_length);
				vector3<double> n_1 = (e1_1*e1_length - e1*e_1) / (e1_length*e1_length);
				vector3<double> n_2 = ( (e1_2*e1_length-e1*e_2)*e1_length - (e1_1*e1_length-e1*e_1)*(2*e_1) ) / (e1_length*e1_length*e1_length);
				T2 = n*d1.d0;
				T2_1 = n*d1.d1 + n_1*d1.d0;
				T2_2 = n*d1.d2 + n_1*(2*d1.d1) + n_2*d1.d0;
			}
			
			vector3<double> e2 = T2 - this->x2;
			vector3<double> e2_1 = T2_1 - this->x3;
			double e2_length = safe_sqrt(e2.get_square());
			e_1_n = e2.x*e2_1.x + e2.y*e2_1.y + e2.z*e2_1.z;
			if( !is_zero(e2_length) )
				e_1 = e_1_n / e2_length;
			else
				e_1 = 0;
			smooth_kp_d1 d2 = smooth_kp_1( e2_length, e_1 , this->P2, this->r3 );
			vector3<double> T3;
			vector3<double> T3_1;
			if( !is_zero(e2_length*e2_length) )
			{
				vector3<double> n = e2 * (1.0/e2_length);
				vector3<double> n_1 = (e2_1*e2_length - e2*e_1) / (e2_length*e2_length);
				T3 = n*d2.d0;
				T3_1 = n*d2.d1 + n_1*d2.d0;
			}
			T3 += T2_1;
			T3_1 += T2_2;
			
			vector3<double> e3 = T3 - this->x3;
			double e3_length = safe_sqrt(e3.get_square());
			double d3 = smooth_kp_0( e3_length , this->P3, this->r4 );
			T4.zero();
			if( !is_zero(e3_length) )
			{
				vector3<double> n = e3 * (1.0/e3_length);
				T4 = n*d3;
			}
			T4 += T3_1;
			
			this->x1 += this->x2*h;
			this->x2 += this->x3*h;
			this->x3 += T4*h;
			
			return this->x1;
		}
		
		inline vector3<double> track2( vector3<double> expect , double h )
		{
			this->tracking_mode = 2;
			
			double e_1_n;
			double e_1;
			
			vector3<double> e2 = expect - this->x2;
			vector3<double> e2_1 = - this->x3;
			double e2_length = safe_sqrt(e2.get_square());
			e_1_n = e2.x*e2_1.x + e2.y*e2_1.y + e2.z*e2_1.z;
			if( !is_zero(e2_length) )
				e_1 = e_1_n / e2_length;
			else
				e_1 = 0;
			smooth_kp_d1 d2 = smooth_kp_1( e2_length, e_1 , this->P2, this->r3 );
			vector3<double> T3;
			vector3<double> T3_1;
			if( !is_zero(e2_length*e2_length) )
			{
				vector3<double> n = e2 * (1.0/e2_length);
				vector3<double> n_1 = (e2_1*e2_length - e2*e_1) / (e2_length*e2_length);
				T3 = n*d2.d0;
				T3_1 = n*d2.d1 + n_1*d2.d0;
			}
			
			vector3<double> e3 = T3 - this->x3;
			double e3_length = safe_sqrt(e3.get_square());
			double d3 = smooth_kp_0( e3_length , this->P3, this->r4 );
			T4.zero();
			if( !is_zero(e3_length) )
			{
				vector3<double> n = e3 * (1.0/e3_length);
				T4 = n*d3;
			}
			T4 += T3_1;
			
			this->x1 += this->x2*h;
			this->x2 += this->x3*h;
			this->x3 += T4*h;
			
			return this->x2;
		}	
};

class TD3_2DSL
{
	private:
		unsigned char tracking_mode;	
	
	public:
		vector2<double> x1;
		vector2<double> x2;
		vector2<double> x3;
		vector2<double> T4;
	
		double P1;
		double P2;
		double P3;
		
		double r2, r3, r4;
		
		inline unsigned char get_tracking_mode(){ return tracking_mode; }
		inline vector2<double> get_x1(){ return x1; }
		inline vector2<double> get_x2(){ return x2; }
		inline vector2<double> get_x3(){ return x3; }
	
		inline void reset()
		{
			this->x1.zero();
			this->x2.zero();
			this->x3.zero();
			tracking_mode = 0;
		}
		
		inline TD3_2DSL()
		{
			this->r2 = this->r3 = this->r4 = 1e12;			
			this->reset();
		}
		inline TD3_2DSL( double P1, double P2, double P3, double P4 )
		{
			this->P1 = P1;
			this->P2 = P2;
			this->P3 = P3;
			this->r2 = this->r3 = this->r4 = 1e12;			
			
			this->reset();
		}
		
		inline vector2<double> track3( vector2<double> expect , double h )
		{
			this->tracking_mode = 3;
	
			double e_1_n;
			double e_1;
			double e_2_n;
			double e_2;
			
			vector2<double> e1 = expect - this->x1;
			vector2<double> e1_1 = -this->x2;
			vector2<double> e1_2 = -this->x3;
			double e1_length = safe_sqrt(e1.get_square());
			e_1_n = e1.x*e1_1.x + e1.y*e1_1.y;
			if( !is_zero(e1_length) )
				e_1 = e_1_n / e1_length;
			else
				e_1 = 0;
			e_2_n = ( e1.x*e1_2.x + e1.y*e1_2.y + e1_1.x*e1_1.x + e1_1.y*e1_1.y )*e1_length - e_1*e_1_n;
			if( !is_zero(e1_length*e1_length) )
				e_2 = e_2_n / (e1_length*e1_length);
			else
				e_2 = 0;
			smooth_kp_d2 d1 = smooth_kp_2( e1_length, e_1, e_2 , this->P1, this->r2 );
			vector2<double> T2;
			vector2<double> T2_1;
			vector2<double> T2_2;
			if( !is_zero(e1_length*e1_length*e1_length) )
			{
				vector2<double> n = e1 * (1.0/e1_length);
				vector2<double> n_1 = (e1_1*e1_length - e1*e_1) / (e1_length*e1_length);
				vector2<double> n_2 = ( (e1_2*e1_length-e1*e_2)*e1_length - (e1_1*e1_length-e1*e_1)*(2*e_1) ) / (e1_length*e1_length*e1_length);
				T2 = n*d1.d0;
				T2_1 = n*d1.d1 + n_1*d1.d0;
				T2_2 = n*d1.d2 + n_1*(2*d1.d1) + n_2*d1.d0;
			}
			
			vector2<double> e2 = T2 - this->x2;
			vector2<double> e2_1 = T2_1 - this->x3;
			double e2_length = safe_sqrt(e2.get_square());
			e_1_n = e2.x*e2_1.x + e2.y*e2_1.y;
			if( !is_zero(e2_length) )
				e_1 = e_1_n / e2_length;
			else
				e_1 = 0;
			smooth_kp_d1 d2 = smooth_kp_1( e2_length, e_1 , this->P2, this->r3 );
			vector2<double> T3;
			vector2<double> T3_1;
			if( !is_zero(e2_length*e2_length) )
			{
				vector2<double> n = e2 * (1.0/e2_length);
				vector2<double> n_1 = (e2_1*e2_length - e2*e_1) / (e2_length*e2_length);
				T3 = n*d2.d0;
				T3_1 = n*d2.d1 + n_1*d2.d0;
			}
			T3 += T2_1;
			T3_1 += T2_2;
			
			vector2<double> e3 = T3 - this->x3;
			double e3_length = safe_sqrt(e3.get_square());
			double d3 = smooth_kp_0( e3_length , this->P3, this->r4 );
			T4.zero();
			if( !is_zero(e3_length) )
			{
				vector2<double> n = e3 * (1.0/e3_length);
				T4 = n*d3;
			}
			T4 += T3_1;
			
			this->x1 += this->x2*h;
			this->x2 += this->x3*h;
			this->x3 += T4*h;
			
			return this->x1;
		}
		
		inline vector2<double> track2( vector2<double> expect , double h )
		{
			this->tracking_mode = 2;
			
			double e_1_n;
			double e_1;
			
			vector2<double> e2 = expect - this->x2;
			vector2<double> e2_1 = - this->x3;
			double e2_length = safe_sqrt(e2.get_square());
			e_1_n = e2.x*e2_1.x + e2.y*e2_1.y;
			if( !is_zero(e2_length) )
				e_1 = e_1_n / e2_length;
			else
				e_1 = 0;
			smooth_kp_d1 d2 = smooth_kp_1( e2_length, e_1 , this->P2, this->r3 );
			vector2<double> T3;
			vector2<double> T3_1;
			if( !is_zero(e2_length*e2_length) )
			{
				vector2<double> n = e2 * (1.0/e2_length);
				vector2<double> n_1 = (e2_1*e2_length - e2*e_1) / (e2_length*e2_length);
				T3 = n*d2.d0;
				T3_1 = n*d2.d1 + n_1*d2.d0;
			}
			
			vector2<double> e3 = T3 - this->x3;
			double e3_length = safe_sqrt(e3.get_square());
			double d3 = smooth_kp_0( e3_length , this->P3, this->r4 );
			T4.zero();
			if( !is_zero(e3_length) )
			{
				vector2<double> n = e3 * (1.0/e3_length);
				T4 = n*d3;
			}
			T4 += T3_1;
			
			this->x1 += this->x2*h;
			this->x2 += this->x3*h;
			this->x3 += T4*h;
			
			return this->x2;
		}
		
};