#pragma once

#include <limits>
#include "arm_math.h"
#include "math_common_tables.hpp"

/*
	基本数学库
	20190909：朱文杰
*/

/*
	常量定义
*/
	#define Pi 3.1415926535897932384626433832795
	#define Pi_f 3.1415926535897932384626433832795f
	
	#define GravityAcc 980.665	//重力加速度(cm/s^2)
	#define rEarth 637139300	//地球半径(cm)
/*常量定义*/

/*ENU和Bodyheading坐标系转换*/
	//ENU坐标转换为BodyHeading（x为机头朝向（与地面平行），y为朝向机头左方（与地面平行），z为上方）
	#define ENU2BodyHeading_x( enu_x , enu_y , Yaw_sin , Yaw_cos ) ( enu_x*Yaw_cos + enu_y*Yaw_sin )
	#define ENU2BodyHeading_y( enu_x , enu_y , Yaw_sin , Yaw_cos ) ( -enu_x*Yaw_sin + enu_y*Yaw_cos )
	//BoduHeading转换为ENU坐标
	#define BodyHeading2ENU_x( body_x , body_y , Yaw_sin , Yaw_cos ) ( body_x*Yaw_cos - body_y*Yaw_sin )
	#define BodyHeading2ENU_y( body_x , body_y , Yaw_sin , Yaw_cos ) ( body_x*Yaw_sin + body_y*Yaw_cos )
/*ENU和Bodyheading坐标系转换*/

/*判断两浮点数是否相等
	v_1:浮点数1
	v_2:浮点数2
*/
	inline bool is_equal(float v_1, float v_2)
	{
		return fabsf(v_1 - v_2) < std::numeric_limits<float>::epsilon();
	}
	inline bool is_equal(double v_1, double v_2)
	{
		return fabs(v_1 - v_2) < std::numeric_limits<double>::epsilon();
	}
/*判断两浮点数是否相等*/
	
/*判断浮点数是否为0
	fVall:浮点数
*/
	inline bool is_zero(float fVal1) 
	{
		return fabsf(fVal1) < std::numeric_limits<float>::epsilon() ? true : false;
	}
	inline bool is_zero(double fVal1) 
	{
		return fabs(fVal1) < std::numeric_limits<double>::epsilon() ? true : false;
	}
/*判断浮点数是否为0*/

/*浮点数取余
	num：被除数
	divider：除数
*/
	inline double Mod( double number, double divider )
	{
		if( is_zero(divider) )
			return 0;
		if( divider < 0 )
			divider = -divider;
		double fraction = number / divider;
		fraction = fraction - (int32_t)fraction;
		return divider*fraction;
	}
	inline float Mod( float number, float divider )
	{
		if( is_zero(divider) )
			return 0;
		if( divider < 0 )
			divider = -divider;
		float fraction = number / divider;
		fraction = fraction - (int32_t)fraction;
		return divider*fraction;
	}
/*浮点数取余*/
	
/*快速三角函数*/
	inline double fast_sin(double theta)
	{
		double fract, tin;                             /* Temporary variables for input, output */
		uint16_t indexS, indexC;                         /* Index variable */
		double f1, f2, d1, d2;                        /* Two nearest output values */
		double findex, Dn, Df, temp;

		/* input x is in degrees */
		/* Scale the input, divide input by 360, for cosine add 0.25 (pi/2) to read sine table */
		tin = theta * 0.15915494309189533576888376337251;

		if (tin < 0.0f)
		{
			tin = -tin;
		}

		tin = tin - (uint32_t)tin;

		/* Calculation of index of the table */
		findex = (double)512 * tin;
		indexS = (uint16_t)(((uint16_t)findex) & 0x1ff);
		indexC = (uint16_t)((indexS + (512 / 4)) & 0x1ff);

		/* fractional value calculation */
		fract = findex - (double)indexS;

		/* Read two nearest values of input value from the cos & sin tables */
		f1 = sinTable_f64[indexS + 0];
		f2 = sinTable_f64[indexS + 1];
		d1 = sinTable_f64[indexC + 0];
		d2 = sinTable_f64[indexC + 1];

		Dn = 0.0122718463030; // delta between the two points (fixed), in this case 2*pi/FAST_MATH_TABLE_SIZE
		Df = f2 - f1; // delta between the values of the functions
		temp = Dn * (d1 + d2) - 2 * Df;
		temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
		temp = fract * temp + d1 * Dn;

		/* Calculation of sine value */
		if (theta < 0.0)
			return -(fract * temp + f1);
		else
			return fract * temp +f1;
	}
	inline double fast_cos(double theta)
	{
		double fract, tin;                             /* Temporary variables for input, output */
		uint16_t indexS, indexC;                         /* Index variable */
		double f1, f2, d1, d2;                        /* Two nearest output values */
		double findex, Dn, Df, temp;

		/* input x is in degrees */
		/* Scale the input, divide input by 360, for cosine add 0.25 (pi/2) to read sine table */
		tin = theta * 0.15915494309189533576888376337251;

		if (tin < 0.0f)
		{
			tin = -tin;
		}

		tin = tin - (uint32_t)tin;

		/* Calculation of index of the table */
		findex = (double)512 * tin;
		indexS = (uint16_t)(((uint16_t)findex) & 0x1ff);
		indexC = (uint16_t)((indexS + (512 / 4)) & 0x1ff);

		/* fractional value calculation */
		fract = findex - (double)indexS;

		/* Read two nearest values of input value from the cos & sin tables */
		f1 = sinTable_f64[indexC + 0];
		f2 = sinTable_f64[indexC + 1];
		d1 = -sinTable_f64[indexS + 0];
		d2 = -sinTable_f64[indexS + 1];

		//return (1.0 - fract) * f1 + fract * f2;

		Dn = 0.0122718463030; // delta between the two points (fixed), in this case 2*pi/FAST_MATH_TABLE_SIZE
		Df = f2 - f1;          // delta between the values of the functions

		temp = Dn * (d1 + d2) - 2 * Df;
		temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
		temp = fract * temp + d1 * Dn;

		/* Calculation of cosine value */
		return fract * temp + f1;
	}
	inline void fast_sin_cos( double theta, double* sin_value, double* cos_value )
	{
		double fract, tin;                             /* Temporary variables for input, output */
		uint16_t indexS, indexC;                         /* Index variable */
		double f1, f2, d1, d2;                        /* Two nearest output values */
		double findex, Dn, Df, temp;

		/* input x is in degrees */
		/* Scale the input, divide input by 360, for cosine add 0.25 (pi/2) to read sine table */
		tin = theta * 0.15915494309189533576888376337251;

		if (tin < 0.0f)
		{
			tin = -tin;
		}

		tin = tin - (uint32_t)tin;

		/* Calculation of index of the table */
		findex = (double)512 * tin;
		indexS = (uint16_t)(((uint16_t)findex) & 0x1ff);
		indexC = (uint16_t)((indexS + (512 / 4)) & 0x1ff);

		/* fractional value calculation */
		fract = findex - (double)indexS;

		/* Read two nearest values of input value from the cos & sin tables */
		f1 = sinTable_f64[indexC + 0];
		f2 = sinTable_f64[indexC + 1];
		d1 = -sinTable_f64[indexS + 0];
		d2 = -sinTable_f64[indexS + 1];

		//return (1.0 - fract) * f1 + fract * f2;

		Dn = 0.0122718463030; // delta between the two points (fixed), in this case 2*pi/FAST_MATH_TABLE_SIZE
		Df = f2 - f1;          // delta between the values of the functions

		temp = Dn * (d1 + d2) - 2 * Df;
		temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
		temp = fract * temp + d1 * Dn;

		/* Calculation of cosine value */
		*cos_value = fract * temp + f1;
		
		/* Read two nearest values of input value from the cos & sin tables */
		f1 = sinTable_f64[indexS + 0];
		f2 = sinTable_f64[indexS + 1];
		d1 = sinTable_f64[indexC + 0];
		d2 = sinTable_f64[indexC + 1];

		Df = f2 - f1; // delta between the values of the functions
		temp = Dn * (d1 + d2) - 2 * Df;
		temp = fract * temp + (3 * Df - (d2 + 2 * d1) * Dn);
		temp = fract * temp + d1 * Dn;

		/* Calculation of sine value */
		if (theta < 0.0)
			*sin_value =  -(fract * temp + f1);
		else
			*sin_value =  fract * temp +f1;
	}
/*快速三角函数*/
	
/*平方函数
	x:数
*/
	template<class T>inline T sq( T data ) 
	{ 
		return data*data;
	}
/*安全开方函数*/
	
/*安全开方函数
	x:浮点数
*/
	inline float safe_sqrt(float x)
	{
		if( x < 0 )
			return 0;
		return sqrtf(x);
	}
	inline double safe_sqrt(double x)
	{
		if( x < 0 )
			return 0;
		return sqrt(x);
	}
/*安全开方函数*/

/*快速exp函数*/
	inline double fast_expd( double x )
	{
		double d;
    *((int*)(&d) + 0) = 0;
    *((int*)(&d) + 1) = (int)(1512775 * x + 1072632447);
    
    double x_ind = x * 1.4426950408889634073599246810019;
    if( x_ind > 0 )
        x_ind -= (int)x_ind;
    else
        x_ind -= (int)x_ind - 1;
    x_ind *= 693;
    int x_ind_int = round( x_ind );
    d *= fast_expd_table[x_ind_int];
            
    return d;
	}
/*快速exp函数*/
	
/*判断数是否在范围内
	x:数
*/
	//判断是否在min-max范围内
	#define in_range( x , min , max ) ( (x<=max) && (x>=min) )
	//判断是否在-range至+range范围内
	#define in_symmetry_range( x , range ) ( ( x >= -range ) && ( x <= range ) )
	//判断是否在mid-range至mid+range范围内
	#define in_symmetry_range_mid( x , mid , range ) ( ( x >= mid - range ) && ( x <= mid + range ) )
/*判断数是否在范围内*/
	
/*移除死区
	x:需要移除死区的数
*/
	//移除-band至+band死区
	template<class T>inline T remove_deadband( T data , T band ) 
	{ 
		if( data > band ) 
			return data-band;
		else if( data < -band ) 
			return data+band;	
		else 
			return 0;
	}
/*移除死区*/

/*限幅
	x:需要被限幅的数
*/
	//min-max范围限幅
	template<class T> inline T constrain( const T x , const T min , const T max )
	{
		if( x < min )
			return min;
		else if( x > max )
			return max;
		else return x;
	}
	
	//-max至+max范围限幅
	template<class T> inline T constrain( const T x , const T max )
	{
		if( x < -max )
			return -max;
		else if( x > max )
			return max;
		else return x;
	}
/*限幅*/
	
/*弧度角度单位转换
	x:需要被转换的数
*/
	inline float rad2degree( float x ) { return x*57.295779513f; }
	inline double rad2degree( double x ) { return x*57.295779513082320876798154814105; }
	inline float degree2rad( float x ) { return x*0.0174532925f; }
	inline double degree2rad( double x ) { return x*0.01745329251994329576923690768489; }
/*弧度角度单位转换*/

/*Ascii码判断转换
	x:Ascii码
*/
	//是否大写字母
	#define is_capital_letter(x) ( x>=65 && x<=90 )
	//是否小写字母
	#define is_lowercase_letter(x) ( x>=97 && x<=122 )
	//是否数字
	#define is_number(x) ( x>=48 && x<=57 )
	//Ascii数字转实际数字
	#define Ascii2num(x) (x-48)
/*Ascii码判断转换*/
	
/*符号函数
	x:数
*/
	template<class T> inline T sign( T x )
	{
		if( x > 0 ) return 1;
		else if( x < 0 ) return -1;
		else return 0;
	}
/*符号函数*/
	
/*向量限幅
	x y z:向量元素
	length:限幅长度
*/
	/*二维向量*/
		inline void constrain_vector( float& x , float& y , float length )
		{
			if( length <= 0.0f ) return;
			float vec_length = safe_sqrt( x*x + y*y );
			if( vec_length > length )
			{
				vec_length = length / vec_length;
				x *= vec_length;
				y *= vec_length;
			}
		}
		inline void constrain_vector( double& x , double& y , double length )
		{
			if( length <= 0.0 ) return;
			double vec_length = safe_sqrt( x*x + y*y );
			if( vec_length > length )
			{
				vec_length = length / vec_length;
				x *= vec_length;
				y *= vec_length;
			}
		}
	/*二维向量*/
		
	/*三维向量*/
		inline void constrain_vector( float& x , float& y , float& z , float length )
		{
			if( length <= 0.0f ) return;
			float vec_length = safe_sqrt( x*x + y*y + z*z );
			if( vec_length > length )
			{
				vec_length = length / vec_length;
				x *= vec_length;
				y *= vec_length;
				z *= vec_length;
			}
		}
		inline void constrain_vector( double& x , double& y , double& z , double length )
		{
			if( length <= 0.0 ) return;
			float vec_length = safe_sqrt( x*x + y*y + z*z );
			if( vec_length > length )
			{
				vec_length = length / vec_length;
				x *= vec_length;
				y *= vec_length;
				z *= vec_length;
			}
		}
	/*三维向量*/
/*向量限幅*/
	
/*nxn矩阵取反
	a:nxn矩阵
*/	
	inline bool Matrix_Inverse(float a[], unsigned char n)  
	{  
			int *is,*js,i,j,k,l,u,v;  
			float d,p;  
			is = new int[n];  
			js = new int[n];  
			for (k=0; k<=n-1; k++)  
			{  
					d=0.0f;  
					for (i=k; i<=n-1; ++i)  
					for (j=k; j<=n-1; ++j)  
					{  
							l=i*n+j; p=fabsf(a[l]);  
							if (p>d) { d=p; is[k]=i; js[k]=j;}  
					}  
					if (is_zero(d))  
					{  
						delete[] is;
						delete[] js; 
							return(false);  
					}  
					if (is[k]!=k)  
							for (j=0; j<=n-1; ++j)  
							{  
									u=k*n+j; v=is[k]*n+j;  
									p=a[u]; a[u]=a[v]; a[v]=p;  
							}  
					if (js[k]!=k)  
							for (i=0; i<=n-1; ++i)  
							{  
									u=i*n+k; v=i*n+js[k];  
									p=a[u]; a[u]=a[v]; a[v]=p;  
							}  
					l=k*n+k;  
					a[l]=1.0f/a[l];  
					for (j=0; j<=n-1; ++j)  
							if (j!=k)  
							{ u=k*n+j; a[u]=a[u]*a[l];}  
					for (i=0; i<=n-1; ++i)  
							if (i!=k)  
									for (j=0; j<=n-1; ++j)  
					if (j!=k)  
					{  
							u=i*n+j;  
							a[u] -= a[i*n+k]*a[k*n+j];  
					}  
					for (i=0; i<=n-1; ++i)  
							if (i!=k)  
							{  
									u=i*n+k;  
									a[u] = -a[u]*a[l];  
							}  
			}  
			for (k=n-1; k>=0; --k)  
			{  
					if (js[k]!=k)  
					for (j=0; j<=n-1; ++j)  
					{  
							u=k*n+j; v=js[k]*n+j;  
					p=a[u]; a[u]=a[v]; a[v]=p;  
					}  
					if (is[k]!=k)  
					for (i=0; i<=n-1; ++i)  
					{   
							u=i*n+k; v=i*n+is[k];  
							p=a[u]; a[u]=a[v]; a[v]=p;  
					}  
			}  
			delete[] is;
			delete[] js;   
			return(true);  
	}
	
	inline bool Matrix_Inverse(double a[], unsigned char n)  
	{  
			int *is,*js,i,j,k,l,u,v;  
			double d,p;  
			is = new int[n];  
			js = new int[n];  
			for (k=0; k<=n-1; k++)  
			{  
					d=0.0f;  
					for (i=k; i<=n-1; ++i)  
					for (j=k; j<=n-1; ++j)  
					{  
							l=i*n+j; p=fabs(a[l]);  
							if (p>d) { d=p; is[k]=i; js[k]=j;}  
					}  
					if (is_zero(d))  
					{  
						delete[] is;
						delete[] js; 
							return(false);  
					}  
					if (is[k]!=k)  
							for (j=0; j<=n-1; ++j)  
							{  
									u=k*n+j; v=is[k]*n+j;  
									p=a[u]; a[u]=a[v]; a[v]=p;  
							}  
					if (js[k]!=k)  
							for (i=0; i<=n-1; ++i)  
							{  
									u=i*n+k; v=i*n+js[k];  
									p=a[u]; a[u]=a[v]; a[v]=p;  
							}  
					l=k*n+k;  
					a[l]=1.0f/a[l];  
					for (j=0; j<=n-1; ++j)  
							if (j!=k)  
							{ u=k*n+j; a[u]=a[u]*a[l];}  
					for (i=0; i<=n-1; ++i)  
							if (i!=k)  
									for (j=0; j<=n-1; ++j)  
					if (j!=k)  
					{  
							u=i*n+j;  
							a[u] -= a[i*n+k]*a[k*n+j];  
					}  
					for (i=0; i<=n-1; ++i)  
							if (i!=k)  
							{  
									u=i*n+k;  
									a[u] = -a[u]*a[l];  
							}  
			}  
			for (k=n-1; k>=0; --k)  
			{  
					if (js[k]!=k)  
					for (j=0; j<=n-1; ++j)  
					{  
							u=k*n+j; v=js[k]*n+j;  
					p=a[u]; a[u]=a[v]; a[v]=p;  
					}  
					if (is[k]!=k)  
					for (i=0; i<=n-1; ++i)  
					{   
							u=i*n+k; v=i*n+is[k];  
							p=a[u]; a[u]=a[v]; a[v]=p;  
					}  
			}  
			delete[] is;
			delete[] js;   
			return(true);  
	}
/*nxn矩阵取反*/