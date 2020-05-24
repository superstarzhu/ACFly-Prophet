#include"quaternion.hpp"

#include "AC_Math.hpp"
#include "vector3.hpp"

/* Quaternion */

	Quaternion Quaternion::get_xy_rotation_quat_zxy() const
	{
		float tqx;
		float tqy;
		float tqw;
		
		float qw2 = this->qw * this->qw;
		float qx2 = this->qx * this->qx;
		float qy2 = this->qy * this->qy;
		float qz2 = this->qz * this->qz;
		float qwx = this->qw * this->qx;
		float qwy = this->qw * this->qy;
		float qwz = this->qw * this->qz;
		float qxy = this->qx * this->qy;
		float qxz = this->qx * this->qz;
		float qyz = this->qy * this->qz;
		
		float qw2Pqz2 = ( qw2 + qz2 );
		if( qw2Pqz2 > std::numeric_limits<float>::min() )
		{		
			tqw = safe_sqrt( qw2Pqz2 );
			float inv_tqw = 1.0f / tqw;
			tqx = ( qwx - qyz ) * inv_tqw;
			tqy = ( qwy + qxz ) * inv_tqw;					
		}
		else
		{
			//glimbal lock effect
			tqw = 0.0f;
			tqx = qx;	tqy = qy;
		}
		return Quaternion( tqw , tqx , tqy , 0 );
	}	
	Quaternion Quaternion::get_xy_rotation_quat_xyz() const
	{
		float tqx;
		float tqy;
		float tqw;
		
		float qw2 = this->qw * this->qw;
		float qx2 = this->qx * this->qx;
		float qy2 = this->qy * this->qy;
		float qz2 = this->qz * this->qz;
		float qwx = this->qw * this->qx;
		float qwy = this->qw * this->qy;
		float qwz = this->qw * this->qz;
		float qxy = this->qx * this->qy;
		float qxz = this->qx * this->qz;
		float qyz = this->qy * this->qz;
		
		float qw2Pqz2 = ( qw2 + qz2 );
		if( qw2Pqz2 > std::numeric_limits<float>::min() )
		{		
			tqw = safe_sqrt( qw2Pqz2 );
			float inv_tqw = 1.0f / tqw;
			tqx = ( qwx + qyz ) * inv_tqw;
			tqy = ( qwy - qxz ) * inv_tqw;					
		}
		else
		{
			//glimbal lock effect
			tqw = 0.0f;
			tqx = qx;	tqy = qy;
		}
		return Quaternion( tqw , tqx , tqy , 0 );
	}	
	vector_3<float> Quaternion::get_Rotation_vec()
	{
		float theta = 2.0f * acosf( this->qw );
		float sin_half_theta = safe_sqrt( 1.0f - this->qw*this->qw );
		float scale;
		if( is_zero( sin_half_theta ) )
			scale = 0.5f;
		else
			scale = theta / sin_half_theta;
		return vector_3<float>( this->qx * scale , this->qy * scale , this->qz * scale );
	}
	
	
	void Quaternion::rotate_delta_angle( const vector_3<float> delta_angle )
	{
		float angle;
		float angle_sin , angle_cosin;
		arm_sqrt_f32( delta_angle.get_square() , &angle );
		float half_angle = angle * 0.5f;
		if( is_zero(half_angle) )
			return;
		arm_sin_cos_f32( rad2degree( half_angle ) , &angle_sin , &angle_cosin );
		
		float qxyz_scale = angle_sin / angle;	
		float Gw = angle_cosin;	float Gx = delta_angle.x * qxyz_scale;	float Gy = delta_angle.y * qxyz_scale;	float Gz = delta_angle.z * qxyz_scale;
		float tqw=qw;	float tqx=qx;	float tqy=qy;	float tqz=qz;
		qw = Gw*tqw - Gx*tqx - Gy*tqy - Gz*tqz;
		qx = Gw*tqx + Gx*tqw + Gy*tqz - Gz*tqy;
		qy = Gw*tqy + Gy*tqw - Gx*tqz + Gz*tqx;
		qz = Gw*tqz + Gx*tqy - Gy*tqx + Gz*tqw;
		normalize();
	}
	
	void Quaternion::integral( vector_3<float> delta_angle0 )
	{
		float tqw=qw;	float tqx=qx;	float tqy=qy;	float tqz=qz;
		qw += 0.5f * ( -tqx*delta_angle0.x - tqy*delta_angle0.y - tqz*delta_angle0.z );
		qx += 0.5f * ( tqw*delta_angle0.x + tqy*delta_angle0.z - tqz*delta_angle0.y );
		qy += 0.5f * ( tqw*delta_angle0.y - tqx*delta_angle0.z + tqz*delta_angle0.x );
		qz += 0.5f * ( tqw*delta_angle0.z + tqx*delta_angle0.y - tqy*delta_angle0.x );
		normalize();
	}
	//integral using RUNGE KUTTA 2
	void Quaternion::integral( vector_3<float> delta_angle0 , vector_3<float> delta_angle1 )
	{
		float qwdx = qw*delta_angle1.x , qwdy = qw*delta_angle1.y , qwdz = qw*delta_angle1.z;
		float qxdx = qx*delta_angle1.x , qxdy = qx*delta_angle1.y , qxdz = qx*delta_angle1.z;
		float qydx = qy*delta_angle1.x , qydy = qy*delta_angle1.y , qydz = qy*delta_angle1.z;
		float qzdx = qz*delta_angle1.x , qzdy = qz*delta_angle1.y , qzdz = qz*delta_angle1.z;
		float qw2 = qw*2.0f , qx2 = qx*2.0f , qy2 = qy*2.0f ,  qz2 = qz*2.0f;
		
		qw += 0.125f*( (qydz - qwdx - qzdy - qx2)*delta_angle0.x + (qzdx - qwdy - qy2 - qxdz)*delta_angle0.y + (qxdy - qydx - qz2 - qwdz)*delta_angle0.z ) - 0.25f*( qxdx + qydy + qzdz );
		qx += 0.125f*( (qw2 - qxdx + qydy + qzdz)*delta_angle0.x + (qwdz - qydx - qxdy - qz2)*delta_angle0.y + (qy2 - qwdy - qzdx - qxdz)*delta_angle0.z ) + 0.25f*( qwdx - qzdy + qydz );
		qy += 0.125f*( (qz2 - qydx - qxdy - qwdz)*delta_angle0.x + (qw2 + qxdx - qydy + qzdz)*delta_angle0.y + (qwdx - qx2 - qzdy - qydz)*delta_angle0.z ) + 0.25f*( qzdx + qwdy - qxdz );
		qz += 0.125f*( (qwdy - qy2 - qzdx - qxdz)*delta_angle0.x + (qx2 - qwdx - qzdy - qydz)*delta_angle0.y + (qw2 + qxdx + qydy - qzdz)*delta_angle0.z ) - 0.25f*( qydx - qxdy - qwdz );
		
		normalize();
	}
	
/* Quaternion */


/* Quaternion_Ef */

	vector_3<float> Quaternion_Ef::rotate( vector_3<float> vec ) const
	{
		return vector_3<float>( \
			(qw2 + qx2 - qy2 - qz2)*vec.x + (2*qxy - 2*qwz)*vec.y + (2*qwy + 2*qxz)*vec.z ,\
			(2*qwz + 2*qxy)*vec.x + (qw2 - qx2 + qy2 - qz2)*vec.y + (2*qyz - 2*qwx)*vec.z ,\
			(2*qxz - 2*qwy)*vec.x + (2*qwx + 2*qyz)*vec.y + (qw2 - qx2 - qy2 + qz2)*vec.z );
	}
	vector_3<float> Quaternion_Ef::rotate_axis_x() const
	{
		return vector_3<float>( \
			qw2 + qx2 - qy2 - qz2 ,\
			2*qwz + 2*qxy ,\
			2*qxz - 2*qwy );
	}
	vector_3<float> Quaternion_Ef::rotate_axis_y() const
	{
		return vector_3<float>( \
			2*qxy - 2*qwz ,\
			qw2 - qx2 + qy2 - qz2 ,\
			2*qwx + 2*qyz );
	}
	vector_3<float> Quaternion_Ef::rotate_axis_z() const
	{
		return vector_3<float>( \
			2*qwy + 2*qxz ,\
			2*qyz - 2*qwx ,\
			qw2 - qx2 - qy2 + qz2 );
	}


	vector_3<float> Quaternion_Ef::reverse_rotate( vector_3<float> vec ) const
	{
		return vector_3<float>( \
			(qw2 + qx2 - qy2 - qz2)*vec.x + (2*qwz + 2*qxy)*vec.y + (2*qxz - 2*qwy)*vec.z ,\
			(2*qxy - 2*qwz)*vec.x + (qw2 - qx2 + qy2 - qz2)*vec.y + (2*qwx + 2*qyz)*vec.z ,\
			(2*qwy + 2*qxz)*vec.x + (2*qyz - 2*qwx)*vec.y + (qw2 - qx2 - qy2 + qz2)*vec.z );
	}
	vector_3<float> Quaternion_Ef::reverse_rotate_axis_x() const
	{
		return vector_3<float>( \
			qw2 + qx2 - qy2 - qz2 ,\
			2*qxy - 2*qwz ,\
			2*qwy + 2*qxz );
	}
	vector_3<float> Quaternion_Ef::reverse_rotate_axis_y() const
	{
		return vector_3<float>( \
			2*qwz + 2*qxy ,\
			qw2 - qx2 + qy2 - qz2 ,\
			2*qyz - 2*qwx );
	}
	vector_3<float> Quaternion_Ef::reverse_rotate_axis_z() const
	{
		return vector_3<float>( \
			2*qxz - 2*qwy ,\
			2*qwx + 2*qyz ,\
			qw2 - qx2 - qy2 + qz2 );
	}

	void Quaternion_Ef::rotate_delta_angle( const vector_3<float> delta_angle )
	{
		float angle;
		float angle_sin , angle_cosin;
		arm_sqrt_f32( delta_angle.get_square() , &angle );
		float half_angle = angle * 0.5f;
		if( is_zero( half_angle ) )
			return;
		arm_sin_cos_f32( rad2degree( half_angle ) , &angle_sin , &angle_cosin );
		
		float qxyz_scale = angle_sin / angle;	
		float Gw = angle_cosin;	float Gx = delta_angle.x * qxyz_scale;	float Gy = delta_angle.y * qxyz_scale;	float Gz = delta_angle.z * qxyz_scale;
//		float norm = Gw*Gw + Gx*Gx + Gy*Gy + Gz*Gz;
//		arm_sqrt_f32( norm , &norm );
//		norm = 1.0f / norm;
//		Gw *= norm;	Gx *= norm;	Gy *= norm;	Gz *= norm;
		float tqw=qw;	float tqx=qx;	float tqy=qy;	float tqz=qz;
		qw = Gw*tqw - Gx*tqx - Gy*tqy - Gz*tqz;
		qx = Gw*tqx + Gx*tqw + Gy*tqz - Gz*tqy;
		qy = Gw*tqy + Gy*tqw - Gx*tqz + Gz*tqx;
		qz = Gw*tqz + Gx*tqy - Gy*tqx + Gz*tqw;
		
		normalize();
		calculate_effeciency_variables();
	}
	
	//integral using RUNGE KUTTA 1
	void Quaternion_Ef::integral( vector_3<float> delta_angle0 )
	{	
		float tqw=qw;	float tqx=qx;	float tqy=qy;	float tqz=qz;
		qw += 0.5f * ( -tqx*delta_angle0.x - tqy*delta_angle0.y - tqz*delta_angle0.z );
		qx += 0.5f * ( tqw*delta_angle0.x + tqy*delta_angle0.z - tqz*delta_angle0.y );
		qy += 0.5f * ( tqw*delta_angle0.y - tqx*delta_angle0.z + tqz*delta_angle0.x );
		qz += 0.5f * ( tqw*delta_angle0.z + tqx*delta_angle0.y - tqy*delta_angle0.x );
		normalize();
		calculate_effeciency_variables();
	}
	//integral using RUNGE KUTTA 2
	void Quaternion_Ef::integral( vector_3<float> delta_angle0 , vector_3<float> delta_angle1 )
	{
		float qwdx = qw*delta_angle1.x , qwdy = qw*delta_angle1.y , qwdz = qw*delta_angle1.z;
		float qxdx = qx*delta_angle1.x , qxdy = qx*delta_angle1.y , qxdz = qx*delta_angle1.z;
		float qydx = qy*delta_angle1.x , qydy = qy*delta_angle1.y , qydz = qy*delta_angle1.z;
		float qzdx = qz*delta_angle1.x , qzdy = qz*delta_angle1.y , qzdz = qz*delta_angle1.z;
		float qw2 = qw*2.0f , qx2 = qx*2.0f , qy2 = qy*2.0f ,  qz2 = qz*2.0f;
		
		qw += 0.125f*( (qydz - qwdx - qzdy - qx2)*delta_angle0.x + (qzdx - qwdy - qy2 - qxdz)*delta_angle0.y + (qxdy - qydx - qz2 - qwdz)*delta_angle0.z ) - 0.25f*( qxdx + qydy + qzdz );
		qx += 0.125f*( (qw2 - qxdx + qydy + qzdz)*delta_angle0.x + (qwdz - qydx - qxdy - qz2)*delta_angle0.y + (qy2 - qwdy - qzdx - qxdz)*delta_angle0.z ) + 0.25f*( qwdx - qzdy + qydz );
		qy += 0.125f*( (qz2 - qydx - qxdy - qwdz)*delta_angle0.x + (qw2 + qxdx - qydy + qzdz)*delta_angle0.y + (qwdx - qx2 - qzdy - qydz)*delta_angle0.z ) + 0.25f*( qzdx + qwdy - qxdz );
		qz += 0.125f*( (qwdy - qy2 - qzdx - qxdz)*delta_angle0.x + (qx2 - qwdx - qzdy - qydz)*delta_angle0.y + (qw2 + qxdx + qydy - qzdz)*delta_angle0.z ) - 0.25f*( qydx - qxdy - qwdz );
		
		normalize();
		calculate_effeciency_variables();
	}
	
	
/* Quaternion_Ef */
