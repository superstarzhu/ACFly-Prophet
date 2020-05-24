#pragma once

#include "AC_Math.hpp"

inline double tan_sig_0( double x )
{
	return 2.0 / ( 1.0 + fast_expd(-2*x) ) - 1;
}
inline double tan_sig_1( double d0 )
{
	return 1 - d0*d0;
}
inline double tan_sig_2( double d0 , double d1 )
{
	return -2*d0*d1;
}
inline double tan_sig_3( double d0 , double d1 , double d2 )
{
	return  -2*( d1*d1 + d0*d2 );
}

struct smooth_kp_d1
{
	double d0;
	double d1;
};
struct smooth_kp_d2
{
	double d0;
	double d1;
	double d2;
};
struct smooth_kp_d3
{
	double d0;
	double d1;
	double d2;
	double d3;
};
inline double smooth_kp_0( double e, double P, double r )
{
	return r*tan_sig_0( P/r*e );
}
inline smooth_kp_d1 smooth_kp_1( double e, double e_1, double P, double r )
{
	double d0 = tan_sig_0( P/r*e );
	double d1 = tan_sig_1( d0 );
	smooth_kp_d1 result;
	result.d0 = r*d0;
	result.d1 = P*d1;
	result.d1 = result.d1*e_1;
	return result;
}
inline smooth_kp_d2 smooth_kp_2( double e, double e_1, double e_2, double P, double r )
{
	double d0 = tan_sig_0( P/r*e );
	double d1 = tan_sig_1( d0 );
	double d2 = tan_sig_2( d0 , d1 );
	smooth_kp_d2 result;
	result.d0 = r*d0;
	d1 = P*d1;
	result.d1 = d1*e_1;
	d2 = P*P/r*d2;
	result.d2 = d2*e_1*e_1 + d1*e_2;
	return result;
}
inline smooth_kp_d3 smooth_kp_3( double e, double e_1, double e_2, double e_3, double P, double r )
{
	double d0 = tan_sig_0( P/r*e );
	double d1 = tan_sig_1( d0 );
	double d2 = tan_sig_2( d0 , d1 );
	double d3 = tan_sig_3( d0 , d1 , d2 );
	smooth_kp_d3 result;
	result.d0 = r*d0;
	d1 = P*d1;
	result.d1 = d1*e_1;
	d2 = P*P/r*d2;
	result.d2 = d2*e_1*e_1 + d1*e_2;
	d3 = P*P*P/(r*r)*d3;
	result.d3 = d3*e_1*e_1*e_1 + 3*d2*e_1*e_2 + d1*e_3;
	return result;
}