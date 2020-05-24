#pragma once

class TD4
{
	private:
		unsigned char tracking_mode;	
	
	public:
		double x1;
		double x2;
		double x3;
		double x4;
	
		double P1;
		double P2;
		double P3;
		double P4;
		
		double r2p , r2n , r3p , r3n , r4p , r4n;
		
		inline unsigned char get_tracking_mode(){ return tracking_mode; }
		inline double get_x1(){ return x1; }
		inline double get_x2(){ return x2; }
		inline double get_x3(){ return x3; }
		inline double get_x4(){ return x4; }
	
		inline void reset()
		{
			this->x1 = this->x2 = this->x3 = this->x4 = 0;
			tracking_mode = 0;
		}
		
		inline TD4()
		{
			this->r2p = this->r2n = this->r3p = this->r3n = this->r4p = this->r4n = 1e12;			
			this->reset();
		}
		inline TD4( double P1, double P2, double P3, double P4 )
		{
			this->P1 = P1;
			this->P2 = P2;
			this->P3 = P3;
			this->P4 = P4;
			this->r2p = this->r2n = this->r3p = this->r3n = this->r4p = this->r4n = 1e12;
			
			this->reset();
		}
		
		inline double track4( double expect , double h )
		{
			this->tracking_mode = 4;
	
			double e1 = expect - this->x1;
			double e1_1 = -this->x2;
			double e1_2 = -this->x3;
			double e1_3 = -this->x4;
			double T2 = this->P1 * e1;
			double P1 = 0;
			if( T2 > this->r2p )
				T2 = this->r2p;
			else if( T2 < -this->r2n )
				T2 = -this->r2n;
			else
				P1 = this->P1;
			double T2_1 = P1 * e1_1;
			double T2_2 = P1 * e1_2;
			double T2_3 = P1 * e1_3;
			
			double e2 = T2 - this->x2;
			double e2_1 = T2_1-this->x3;
			double e2_2 = T2_2-this->x4;
			double T3 = this->P2 * e2;
			double P2 = 0;
			if( T3 > this->r3p )
				T3 = this->r3p;
			else if( T3 < -this->r3n )
				T3 = -this->r3n;
			else
				P2 = this->P2;
			T3 += T2_1;
			double T3_1 = P2 * e2_1 + T2_2;
			double T3_2 = P2 * e2_2 + T2_3;
			
			double e3 = T3 - this->x3;
			double e3_1 = T3_1-this->x4;
			double T4 = this->P3 * e3;
			double P3 = 0;
			if( T4 > this->r4p )
				T4 = this->r4p;
			else if( T4 < -this->r4n )
				T4 = -this->r4n;
			else
				P3 = this->P3;
			T4 += T3_1;
			double T4_1 = P3 * e3_1 + T3_2;
			
			double e4 = T4 - this->x4;
			double T5 = this->P4 * e4 + T4_1;
			
			this->x1 += h*this->x2;
			this->x2 += h*this->x3;
			this->x3 += h*this->x4;
			this->x4 += h*T5;
			
			return this->x1;
		}
		
		inline double track3( double expect , double h )
		{
			this->tracking_mode = 3;
	
			double e2 = expect - this->x2;
			double e2_1 = -this->x3;
			double e2_2 = -this->x4;
			double T3 = this->P2 * e2;
			double P2 = 0;
			if( T3 > this->r3p )
				T3 = this->r3p;
			else if( T3 < -this->r3n )
				T3 = -this->r3n;
			else
				P2 = this->P2;
			double T3_1 = P2 * e2_1;
			double T3_2 = P2 * e2_2;
			
			double e3 = T3 - this->x3;
			double e3_1 = T3_1-this->x4;
			double T4 = this->P3 * e3;
			double P3 = 0;
			if( T4 > this->r4p )
				T4 = this->r4p;
			else if( T4 < -this->r4n )
				T4 = -this->r4n;
			else
				P3 = this->P3;
			T4 += T3_1;
			double T4_1 = P3 * e3_1 + T3_2;
			
			double e4 = T4 - this->x4;
			double T5 = this->P4 * e4 + T4_1;
			
			this->x2 += h*this->x3;
			this->x3 += h*this->x4;
			this->x4 += h*T5;
			
			return this->x2;
		}
		
};

class TD4_Lite
{
	private:
		unsigned char tracking_mode;	
	
	public:
		double x1;
		double x2;
		double x3;
		double x4;
		
		inline unsigned char get_tracking_mode(){ return tracking_mode; }
		inline double get_x1(){ return x1; }
		inline double get_x2(){ return x2; }
		inline double get_x3(){ return x3; }
		inline double get_x4(){ return x4; }
	
		inline void reset()
		{
			this->x1 = this->x2 = this->x3 = this->x4 = 0;
			tracking_mode = 0;
		}
		
		inline TD4_Lite()
		{		
			this->reset();
		}
		
		inline double track4( double expect , double h , double P1 , double P2 , double P3 , double P4 )
		{
			this->tracking_mode = 4;
	
			double e1 = expect - this->x1;
			double e1_1 = -this->x2;
			double e1_2 = -this->x3;
			double e1_3 = -this->x4;
			double T2 = P1 * e1;
			double T2_1 = P1 * e1_1;
			double T2_2 = P1 * e1_2;
			double T2_3 = P1 * e1_3;
			
			double e2 = T2 - this->x2;
			double e2_1 = T2_1-this->x3;
			double e2_2 = T2_2-this->x4;
			double T3 = P2 * e2;
			T3 += T2_1;
			double T3_1 = P2 * e2_1 + T2_2;
			double T3_2 = P2 * e2_2 + T2_3;
			
			double e3 = T3 - this->x3;
			double e3_1 = T3_1-this->x4;
			double T4 = P3 * e3;
			T4 += T3_1;
			double T4_1 = P3 * e3_1 + T3_2;
			
			double e4 = T4 - this->x4;
			double T5 = P4 * e4 + T4_1;
			
			this->x1 += h*this->x2;
			this->x2 += h*this->x3;
			this->x3 += h*this->x4;
			this->x4 += h*T5;
			
			return this->x1;
		}
		
		inline double track3( double expect , double h , double P2 , double P3 , double P4 )
		{
			this->tracking_mode = 3;
	
			double e2 = expect - this->x2;
			double e2_1 = -this->x3;
			double e2_2 = -this->x4;
			double T3 = P2 * e2;
			double T3_1 = P2 * e2_1;
			double T3_2 = P2 * e2_2;
			
			double e3 = T3 - this->x3;
			double e3_1 = T3_1-this->x4;
			double T4 = P3 * e3;
			T4 += T3_1;
			double T4_1 = P3 * e3_1 + T3_2;
			
			double e4 = T4 - this->x4;
			double T5 = P4 * e4 + T4_1;
			
			this->x2 += h*this->x3;
			this->x3 += h*this->x4;
			this->x4 += h*T5;
			
			return this->x2;
		}
		
};