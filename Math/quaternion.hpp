#pragma once

#include "AC_Math.hpp"
#include "vector3.hpp"

/*
	四元数类
	20190909：朱文杰
	未经同意严禁用于商业用途！
*/

class Quaternion
{
	protected:
		double qw , qx , qy , qz;		
	
		//四元数单位化
		inline void normalize()
		{
			double inv_length = safe_sqrt( this->qw*this->qw + this->qx*this->qx + this->qy*this->qy + this->qz*this->qz );
			inv_length = 1.0 / inv_length;
			this->qw *= inv_length;
			this->qx *= inv_length;
			this->qy *= inv_length;
			this->qz *= inv_length;
		}
	
		
	public:
		//构造函数
		inline Quaternion()
		{
			this->qw = 1;
			this->qx = 0;
			this->qy = 0;
			this->qz = 0;
		}
		inline Quaternion( double qw , double qx , double qy , double qz )
		{
			this->qw = qw;
			this->qx = qx;
			this->qy = qy;
			this->qz = qz;
			normalize();
		}
	
		//四元数乘法
		inline Quaternion operator *( const Quaternion& b ) const
		{
			return Quaternion( qw*b.qw - qx*b.qx - qy*b.qy - qz*b.qz,
													qw*b.qx + qx*b.qw + qy*b.qz - qz*b.qy, 
													qw*b.qy + qy*b.qw - qx*b.qz + qz*b.qx, 
													qw*b.qz + qx*b.qy - qy*b.qx + qz*b.qw );
		}
		
		//获取四元数元素值
		inline const double* get() const { return &this->qw; }
		inline double get_qw() const { return this->qw; } 
		inline double get_qx() const { return this->qx; }
		inline double get_qy() const { return this->qy; }
		inline double get_qz() const { return this->qz; }
		
		//分解四元数
		inline Quaternion get_xy_rotation_quat() const
		{
			double tqx;
			double tqy;
			double tqw;
		
			double qw2 = this->qw * this->qw;
			double qx2 = this->qx * this->qx;
			double qy2 = this->qy * this->qy;
			double qz2 = this->qz * this->qz;
			double qwx = this->qw * this->qx;
			double qwy = this->qw * this->qy;
			double qwz = this->qw * this->qz;
			double qxy = this->qx * this->qy;
			double qxz = this->qx * this->qz;
			double qyz = this->qy * this->qz;
			
			double qw2Pqz2 = ( qw2 + qz2 );
			if( !is_zero(qw2Pqz2) )
			{		
				tqw = safe_sqrt( qw2Pqz2 );
				double inv_tqw = 1.0 / tqw;
				tqx = ( qwx + qyz ) * inv_tqw;
				tqy = ( qwy - qxz ) * inv_tqw;					
			}
			else
			{
				//glimbal lock effect
				tqw = 0.0;
				tqx = this->qx;	tqy = this->qy;
			}
			return Quaternion( tqw , tqx , tqy , 0 );
		}
		double getPitch() const
		{
			return asin( 2.0*(qw*qy-qx*qz) );
		}
		double getRoll() const
		{
			return atan2( 2.0*(qw*qx+qy*qz) ,\
				1.0-2.0*(qx*qx+qy*qy) );
		}	
		double getYaw() const
		{
			return atan2( 2.0*(qw*qz+qx*qy) ,\
				1.0-2.0*(qy*qy+qz*qz) );
		}
		void Enu2Ned()
		{
			double tqx = this->qx;
			this->qx = this->qy;
			this->qy = tqx;
			this->qz = -this->qz;
			*this = *this * Quaternion( 0.70710678118654752440084436210485, 0, 0, 0.70710678118654752440084436210485 );
		}
		
		//四元数转旋转向量
		inline vector3<double> get_Rotation_vec()
		{
			double theta = 2.0* acos( this->qw );
			if(theta > Pi)
				theta -= 2*Pi;
			double sin_half_theta = safe_sqrt( 1.0 - this->qw*this->qw );
			double scale;
			if( is_zero( sin_half_theta ) )
				scale = 0.5;
			else
				scale = theta / sin_half_theta;
			return vector3<double>( this->qx * scale , this->qy * scale , this->qz * scale );
		}
		
		//求共轭四元数
		inline void conjugate()
		{
			this->qx = -this->qx;
			this->qy = -this->qy;
			this->qz = -this->qz;
		}
		
		/*用四元数正向旋转向量（QvecQ*）
			vec:被旋转的向量
		*/
			inline vector3<double> rotate( vector3<double> vec ) const
			{
				double qw2 = this->qw * this->qw;
				double qx2 = this->qx * this->qx;
				double qy2 = this->qy * this->qy;
				double qz2 = this->qz * this->qz;
				double qwx = this->qw * this->qx;
				double qwy = this->qw * this->qy;
				double qwz = this->qw * this->qz;
				double qxy = this->qx * this->qy;
				double qxz = this->qx * this->qz;
				double qyz = this->qy * this->qz;
				return vector3<double>( \
					(qw2 + qx2 - qy2 - qz2)*vec.x + (2*qxy - 2*qwz)*vec.y + (2*qwy + 2*qxz)*vec.z ,\
					(2*qwz + 2*qxy)*vec.x + (qw2 - qx2 + qy2 - qz2)*vec.y + (2*qyz - 2*qwx)*vec.z ,\
					(2*qxz - 2*qwy)*vec.x + (2*qwx + 2*qyz)*vec.y + (qw2 - qx2 - qy2 + qz2)*vec.z );
			}
			inline vector3<double> rotate_axis_x() const
			{
				double qw2 = this->qw * this->qw;
				double qx2 = this->qx * this->qx;
				double qy2 = this->qy * this->qy;
				double qz2 = this->qz * this->qz;
				double qwx = this->qw * this->qx;
				double qwy = this->qw * this->qy;
				double qwz = this->qw * this->qz;
				double qxy = this->qx * this->qy;
				double qxz = this->qx * this->qz;
				double qyz = this->qy * this->qz;
				return vector3<double>( \
					qw2 + qx2 - qy2 - qz2 ,\
					2*qwz + 2*qxy ,\
					2*qxz - 2*qwy );
			}
			inline vector3<double> rotate_axis_y() const
			{
				double qw2 = this->qw * this->qw;
				double qx2 = this->qx * this->qx;
				double qy2 = this->qy * this->qy;
				double qz2 = this->qz * this->qz;
				double qwx = this->qw * this->qx;
				double qwy = this->qw * this->qy;
				double qwz = this->qw * this->qz;
				double qxy = this->qx * this->qy;
				double qxz = this->qx * this->qz;
				double qyz = this->qy * this->qz;
				return vector3<double>( \
					2*qxy - 2*qwz ,\
					qw2 - qx2 + qy2 - qz2 ,\
					2*qwx + 2*qyz );
			}
			inline vector3<double> rotate_axis_z() const
			{
				double qw2 = this->qw * this->qw;
				double qx2 = this->qx * this->qx;
				double qy2 = this->qy * this->qy;
				double qz2 = this->qz * this->qz;
				double qwx = this->qw * this->qx;
				double qwy = this->qw * this->qy;
				double qwz = this->qw * this->qz;
				double qxy = this->qx * this->qy;
				double qxz = this->qx * this->qz;
				double qyz = this->qy * this->qz;
				return vector3<double>( \
					2*qwy + 2*qxz ,\
					2*qyz - 2*qwx ,\
					qw2 - qx2 - qy2 + qz2 );
			}
		/*用四元数正向旋转向量（QvecQ*）*/
		
		/*用四元数逆向旋转向量（Q*vecQ）
			vec:被旋转的向量
		*/
			inline vector3<double> reverse_rotate( vector3<double> vec ) const
			{
				double qw2 = this->qw * this->qw;
				double qx2 = this->qx * this->qx;
				double qy2 = this->qy * this->qy;
				double qz2 = this->qz * this->qz;
				double qwx = this->qw * this->qx;
				double qwy = this->qw * this->qy;
				double qwz = this->qw * this->qz;
				double qxy = this->qx * this->qy;
				double qxz = this->qx * this->qz;
				double qyz = this->qy * this->qz;
				return vector3<double>( \
					(qw2 + qx2 - qy2 - qz2)*vec.x + 2*(qwz + qxy)*vec.y + 2*(qxz - qwy)*vec.z ,\
					2*(qxy - qwz)*vec.x + (qw2 - qx2 + qy2 - qz2)*vec.y + 2*(qwx + qyz)*vec.z ,\
					2*(qwy + qxz)*vec.x + 2*(qyz - qwx)*vec.y + (qw2 - qx2 - qy2 + qz2)*vec.z );
			}
			inline vector3<double> reverse_rotate_axis_x() const
			{
				double qw2 = this->qw * this->qw;
				double qx2 = this->qx * this->qx;
				double qy2 = this->qy * this->qy;
				double qz2 = this->qz * this->qz;
				double qwx = this->qw * this->qx;
				double qwy = this->qw * this->qy;
				double qwz = this->qw * this->qz;
				double qxy = this->qx * this->qy;
				double qxz = this->qx * this->qz;
				double qyz = this->qy * this->qz;
				return vector3<double>( \
					qw2 + qx2 - qy2 - qz2 ,\
					2*qxy - 2*qwz ,\
					2*qwy + 2*qxz );
			}
			inline vector3<double> reverse_rotate_axis_y() const
			{
				double qw2 = this->qw * this->qw;
				double qx2 = this->qx * this->qx;
				double qy2 = this->qy * this->qy;
				double qz2 = this->qz * this->qz;
				double qwx = this->qw * this->qx;
				double qwy = this->qw * this->qy;
				double qwz = this->qw * this->qz;
				double qxy = this->qx * this->qy;
				double qxz = this->qx * this->qz;
				double qyz = this->qy * this->qz;
				return vector3<double>( \
					2*qwz + 2*qxy ,\
					qw2 - qx2 + qy2 - qz2 ,\
					2*qyz - 2*qwx );
			}
			inline vector3<double> reverse_rotate_axis_z() const
			{
				double qw2 = this->qw * this->qw;
				double qx2 = this->qx * this->qx;
				double qy2 = this->qy * this->qy;
				double qz2 = this->qz * this->qz;
				double qwx = this->qw * this->qx;
				double qwy = this->qw * this->qy;
				double qwz = this->qw * this->qz;
				double qxy = this->qx * this->qy;
				double qxz = this->qx * this->qz;
				double qyz = this->qy * this->qz;
				return vector3<double>( \
					2*qxz - 2*qwy ,\
					2*qwx + 2*qyz ,\
					qw2 - qx2 - qy2 + qz2 );
			}
		/*用四元数逆向旋转向量（Q*vecQ）*/
			
		//将当前四元数旋转一个角度
		inline void rotate_delta_angle( vector3<double> delta_angle )
		{
			double angle;
			angle = safe_sqrt( delta_angle.get_square() );
			double half_angle = angle * 0.5;
			if( is_zero(half_angle) )
				return;
			double angle_sin, angle_cosin;
			fast_sin_cos( half_angle, &angle_sin, &angle_cosin );
			
			double qxyz_scale = angle_sin / angle;	
			double Gw = angle_cosin;	double Gx = delta_angle.x * qxyz_scale;	double Gy = delta_angle.y * qxyz_scale;	double Gz = delta_angle.z * qxyz_scale;
			double tqw=qw;	double tqx=qx;	double tqy=qy;	double tqz=qz;
			qw = Gw*tqw - Gx*tqx - Gy*tqy - Gz*tqz;
			qx = Gw*tqx + Gx*tqw + Gy*tqz - Gz*tqy;
			qy = Gw*tqy + Gy*tqw - Gx*tqz + Gz*tqx;
			qz = Gw*tqz + Gx*tqy - Gy*tqx + Gz*tqw;
			normalize();
		}
		
		//求旋转矩阵
		inline void get_rotation_matrix(double matrix[3][3]) const
		{
			double qw2 = this->qw * this->qw;
			double qx2 = this->qx * this->qx;
			double qy2 = this->qy * this->qy;
			double qz2 = this->qz * this->qz;
			double qwx = this->qw * this->qx;
			double qwy = this->qw * this->qy;
			double qwz = this->qw * this->qz;
			double qxy = this->qx * this->qy;
			double qxz = this->qx * this->qz;
			double qyz = this->qy * this->qz;
			matrix[0][0]=qw2+qx2-qy2-qz2;	matrix[0][1]=2.0*(qxy-qwz);	matrix[0][2]=2.0*(qwy+qxz);
			matrix[1][0]=2.0*(qwz+qxy);	matrix[1][1]=qw2-qx2+qy2-qz2;	matrix[1][2]=2.0*(qyz-qwx);
			matrix[2][0]=2.0*(qxz-qwy);	matrix[2][1]=2.0*(qwx+qyz);	matrix[2][2]=qw2-qx2-qy2+qz2;
		}
		
		//求倾斜角度cosin
		inline double get_lean_angle_cosin() const
		{
			return  this->qw*this->qw - \
							this->qx*this->qx - \
							this->qy*this->qy + \
							this->qz*this->qz;
		}
			
		//四元数旋转向量静态函数
		static inline vector3<double> rotate_vector( const vector3<double> rotation , const vector3<double> vec )
		{
			double theta = safe_sqrt( rotation.get_square() );
			if( is_zero( theta ) )
				return vec;
			
			double inv_theta = 1.0 / theta;
			double x = rotation.x * inv_theta;
			double y = rotation.y * inv_theta;
			double z = rotation.z * inv_theta;
			double x2 = x*x;	double y2 = y*y;	double z2 = z*z;
			double xy = x*y;	double yz = y*z;	double xz = x*z;
			
			double a = vec.x;	double b = vec.y;	double c = vec.z;
			
			double sin_theta , cos_theta;
			fast_sin_cos( theta, &sin_theta, &cos_theta );
			//arm_sin_cos_f32( rad2degree( theta ) , &sin_theta , &cos_theta );
					
			return vector3<double>( \
				- a*(cos_theta/2 - 1/2)*x2 - b*(cos_theta - 1)*xy - c*(cos_theta - 1)*xz + a*(cos_theta/2 - 1/2)*y2 + c*sin_theta*y + a*(cos_theta/2 - 1/2)*z2 - b*sin_theta*z + a*(cos_theta/2 + 1/2),\
				b*(cos_theta/2 - 1/2)*x2 - a*(cos_theta - 1)*xy - c*sin_theta*x - b*(cos_theta/2 - 1/2)*y2 - c*(cos_theta - 1)*yz + b*(cos_theta/2 - 1/2)*z2 + a*sin_theta*z + b*(cos_theta/2 + 1/2),\
				c*(cos_theta/2 - 1/2)*x2 - a*(cos_theta - 1)*xz + b*sin_theta*x + c*(cos_theta/2 - 1/2)*y2 - b*(cos_theta - 1)*yz - a*sin_theta*y - c*(cos_theta/2 - 1/2)*z2 + c*(cos_theta/2 + 1/2)\
			);
		}
		
		//四元数积分一阶龙格库塔法
		void integral( vector3<double> delta_angle0 )
		{
			double tqw=qw;	double tqx=qx;	double tqy=qy;	double tqz=qz;
			qw += 0.5 * ( -tqx*delta_angle0.x - tqy*delta_angle0.y - tqz*delta_angle0.z );
			qx += 0.5 * ( tqw*delta_angle0.x + tqy*delta_angle0.z - tqz*delta_angle0.y );
			qy += 0.5 * ( tqw*delta_angle0.y - tqx*delta_angle0.z + tqz*delta_angle0.x );
			qz += 0.5 * ( tqw*delta_angle0.z + tqx*delta_angle0.y - tqy*delta_angle0.x );
			normalize();
		}
		//四元数积分二阶龙格库塔法
		void integral( vector3<double> delta_angle0 , vector3<double> delta_angle1 )
		{
			double qwdx = qw*delta_angle1.x , qwdy = qw*delta_angle1.y , qwdz = qw*delta_angle1.z;
			double qxdx = qx*delta_angle1.x , qxdy = qx*delta_angle1.y , qxdz = qx*delta_angle1.z;
			double qydx = qy*delta_angle1.x , qydy = qy*delta_angle1.y , qydz = qy*delta_angle1.z;
			double qzdx = qz*delta_angle1.x , qzdy = qz*delta_angle1.y , qzdz = qz*delta_angle1.z;
			double qw2 = qw*2.0 , qx2 = qx*2.0 , qy2 = qy*2.0 ,  qz2 = qz*2.0;
			
			qw += 0.125*( (qydz - qwdx - qzdy - qx2)*delta_angle0.x + (qzdx - qwdy - qy2 - qxdz)*delta_angle0.y + (qxdy - qydx - qz2 - qwdz)*delta_angle0.z ) - 0.25f*( qxdx + qydy + qzdz );
			qx += 0.125*( (qw2 - qxdx + qydy + qzdz)*delta_angle0.x + (qwdz - qydx - qxdy - qz2)*delta_angle0.y + (qy2 - qwdy - qzdx - qxdz)*delta_angle0.z ) + 0.25f*( qwdx - qzdy + qydz );
			qy += 0.125*( (qz2 - qydx - qxdy - qwdz)*delta_angle0.x + (qw2 + qxdx - qydy + qzdz)*delta_angle0.y + (qwdx - qx2 - qzdy - qydz)*delta_angle0.z ) + 0.25f*( qzdx + qwdy - qxdz );
			qz += 0.125*( (qwdy - qy2 - qzdx - qxdz)*delta_angle0.x + (qx2 - qwdx - qzdy - qydz)*delta_angle0.y + (qw2 + qxdx + qydy - qzdz)*delta_angle0.z ) - 0.25f*( qydx - qxdy - qwdz );
			
			normalize();
		}
};

class Quaternion_Ef:public Quaternion
{
	private:
		//quat related variables for math effeciency
		double qw2 , qx2 , qy2, qz2 , qwx , qwy , qwz , qxy , qxz , qyz;
	
		inline void calculate_effeciency_variables()
		{
			this->qw2 = this->qw * this->qw;
			this->qx2 = this->qx * this->qx;
			this->qy2 = this->qy * this->qy;
			this->qz2 = this->qz * this->qz;
			this->qwx = this->qw * this->qx;
			this->qwy = this->qw * this->qy;
			this->qwz = this->qw * this->qz;
			this->qxy = this->qx * this->qy;
			this->qxz = this->qx * this->qz;
			this->qyz = this->qy * this->qz;
		}
	
	public:	
		
		//构造函数
		inline Quaternion_Ef( const Quaternion* quat ):Quaternion( quat->get_qw() , quat->get_qx() , quat->get_qy() , quat->get_qz() )
		{
			calculate_effeciency_variables();
		}
		inline Quaternion_Ef( const Quaternion& quat ):Quaternion( quat.get_qw() , quat.get_qx() , quat.get_qy() , quat.get_qz() )
		{
			calculate_effeciency_variables();
		}
		inline Quaternion_Ef( double qw , double qx , double qy , double qz ):Quaternion( qw , qx , qy , qz )
		{			
			calculate_effeciency_variables();
		}	
		inline Quaternion_Ef():Quaternion()
		{			
			calculate_effeciency_variables();
		}	
	
		inline void conjugate()
		{
			this->qx = -this->qx;
			this->qy = -this->qy;
			this->qz = -this->qz;
			calculate_effeciency_variables();
		}
		
		//获取四元数效率元素
		inline double get_qw2() const { return qw2; }
		inline double get_qx2() const { return qx2; }
		inline double get_qy2() const { return qy2; }
		inline double get_qz2() const { return qz2; }
		inline double get_qwx() const { return qwx; }
		inline double get_qwy() const { return qwy; }
		inline double get_qwz() const { return qwz; }
		inline double get_qxy() const { return qxy; }
		inline double get_qxz() const { return qxz; }
		inline double get_qyz() const { return qyz; }
		
		//四元数分解
		inline Quaternion get_xy_rotation_quat() const
		{
			double tqx;
			double tqy;
			double tqw;
			
			double qw2Pqz2 = ( qw2 + qz2 );
			if( !is_zero(qw2Pqz2) )
			{		
				tqw = safe_sqrt( qw2Pqz2 );
				double inv_tqw = 1.0 / tqw;
				tqx = ( this->qwx + this->qyz ) * inv_tqw;
				tqy = ( this->qwy - this->qxz ) * inv_tqw;					
			}
			else
			{
				//glimbal lock effect
				tqw = 0.0;
				tqx = this->qx;	tqy = this->qy;
			}
			return Quaternion( tqw , tqx , tqy , 0 );
		}

		/*用四元数正向旋转向量（QvecQ*）
			vec:被旋转的向量
		*/
			inline vector3<double> rotate( vector3<double> vec ) const
			{
				return vector3<double>( \
					(qw2 + qx2 - qy2 - qz2)*vec.x + (2*qxy - 2*qwz)*vec.y + (2*qwy + 2*qxz)*vec.z ,\
					(2*qwz + 2*qxy)*vec.x + (qw2 - qx2 + qy2 - qz2)*vec.y + (2*qyz - 2*qwx)*vec.z ,\
					(2*qxz - 2*qwy)*vec.x + (2*qwx + 2*qyz)*vec.y + (qw2 - qx2 - qy2 + qz2)*vec.z );
			}
			inline vector3<double> rotate_axis_x() const
			{
				return vector3<double>( \
					qw2 + qx2 - qy2 - qz2 ,\
					2*qwz + 2*qxy ,\
					2*qxz - 2*qwy );
			}
			inline vector3<double> rotate_axis_y() const
			{
				return vector3<double>( \
					2*qxy - 2*qwz ,\
					qw2 - qx2 + qy2 - qz2 ,\
					2*qwx + 2*qyz );
			}
			inline vector3<double> rotate_axis_z() const
			{
				return vector3<double>( \
					2*qwy + 2*qxz ,\
					2*qyz - 2*qwx ,\
					qw2 - qx2 - qy2 + qz2 );
			}
		/*用四元数正向旋转向量（QvecQ*）*/
		
		/*用四元数逆向旋转向量（Q*vecQ）
			vec:被旋转的向量
		*/
			inline vector3<double> reverse_rotate( vector3<double> vec ) const
			{
				return vector3<double>( \
					(qw2 + qx2 - qy2 - qz2)*vec.x + 2*(qwz + qxy)*vec.y + 2*(qxz - qwy)*vec.z ,\
					2*(qxy - qwz)*vec.x + (qw2 - qx2 + qy2 - qz2)*vec.y + 2*(qwx + qyz)*vec.z ,\
					2*(qwy + qxz)*vec.x + 2*(qyz - qwx)*vec.y + (qw2 - qx2 - qy2 + qz2)*vec.z );
			}
			inline vector3<double> reverse_rotate_axis_x() const
			{
				return vector3<double>( \
					qw2 + qx2 - qy2 - qz2 ,\
					2*qxy - 2*qwz ,\
					2*qwy + 2*qxz );
			}
			inline vector3<double> reverse_rotate_axis_y() const
			{
				return vector3<double>( \
					2*qwz + 2*qxy ,\
					qw2 - qx2 + qy2 - qz2 ,\
					2*qyz - 2*qwx );
			}
			inline vector3<double> reverse_rotate_axis_z() const
			{
				return vector3<double>( \
					2*qxz - 2*qwy ,\
					2*qwx + 2*qyz ,\
					qw2 - qx2 - qy2 + qz2 );
			}
		/*用四元数逆向旋转向量（Q*vecQ）*/
		
		//求倾斜角度cosin
		inline double get_lean_angle_cosin() const
		{
			return  this->qw2 - \
							this->qx2 - \
							this->qy2 + \
							this->qz2;
		}
		
		//求旋转矩阵
		inline void get_rotation_matrix(double matrix[3][3]) const
		{
			matrix[0][0]=qw2+qx2-qy2-qz2;	matrix[0][1]=2.0*(qxy-qwz);	matrix[0][2]=2.0*(qwy+qxz);
			matrix[1][0]=2.0*(qwz+qxy);	matrix[1][1]=qw2-qx2+qy2-qz2;	matrix[1][2]=2.0*(qyz-qwx);
			matrix[2][0]=2.0*(qxz-qwy);	matrix[2][1]=2.0*(qwx+qyz);	matrix[2][2]=qw2-qx2-qy2+qz2;
		}
			
		inline void rotate_delta_angle( vector3<double> delta_angle )
		{
			Quaternion::rotate_delta_angle(delta_angle);
			calculate_effeciency_variables();
		}
		
		//四元数积分一阶龙格库塔法
		void integral( vector3<double> delta_angle0 )
		{
			Quaternion::integral(delta_angle0);
			calculate_effeciency_variables();
		}
		//四元数积分二阶龙格库塔法
		void integral( vector3<double> delta_angle0 , vector3<double> delta_angle1 )
		{			
			Quaternion::integral(delta_angle0,delta_angle1);
			calculate_effeciency_variables();
		}
};
