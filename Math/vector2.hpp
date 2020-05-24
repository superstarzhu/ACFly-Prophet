#pragma once

#include "AC_Math.hpp"

/*
	三维向量类
	20190909：朱文杰
*/

template<typename T>
class vector2
{
	private:
		
	public:
		T x , y;
	
		/*构造函数
			x y：向量三维分量
		*/
			inline vector2(){this->x = this->y = 0;}
			inline vector2( T x , T y )
			{
				this->x = x;
				this->y = y;
			}
			inline void set_vector( const T x , const T y )
			{
				this->x = x;
				this->y = y;
			}
		/*构造函数*/
		
		//向量清零
		inline void zero()
		{
			x=0;	y=0;
		}
		
		//等号运算符重载
		inline vector2<T>& operator =( const vector2<T>& b )
		{
			x=b.x;	y=b.y;
			return *this;
		}
		
		//下标运算符重载
		inline T& operator[](int i)
		{
			if( i == 0 ) return x;
			else return y;
		}
		
		/*向量与向量、标量基本运算符重载
			向量*向量：点乘
			向量%向量：叉乘
			向量&向量：元素各自相乘
		*/
			inline vector2<T> operator +( const vector2<T>& b ) const
			{
				return vector2<T>( x+b.x , y+b.y );
			}
			inline vector2<T> operator -( const vector2<T>& b ) const
			{
				return vector2<T>( x-b.x , y-b.y );
			}
			inline vector2<T> operator *( const T b ) const
			{
				return vector2<T>( x*b , y*b );
			}
			inline vector2<T> operator /( const T b ) const
			{
				return vector2<T>( x/b , y/b );
			}
			inline vector2<T> operator -(void) const
			{
				return vector2<T>( -x , -y );
			}
			
			inline vector2<T> operator +=( const vector2<T>& b )
			{
				x += b.x;
				y += b.y;
				return *this;
			}
			
			inline vector2<T> operator -=( const vector2<T>& b )
			{
				this->x -= b.x;
				this->y -= b.y;
				return *this;
			}
			inline vector2<T> operator *=( T b )
			{
				this->x *= b;
				this->y *= b;
				return *this;
			}
			
			//点乘
			inline T operator *( const vector2<T>& b )const
			{
				return this->x*b.x + this->y*b.y;
			}		
			//叉乘
			inline vector2<T> operator %( const vector2<T>& b )const
			{
				vector2<T> result;
				result.x = this->y * b.x - this->x *b.y;
				result.y = this->x * b.y - this->y *b.x;
				return result;
			}
			//元素各自相乘
			inline vector2<T> operator &( const vector2<T>& b )const
			{
				vector2<T> result;
				result.x = this->x * b.x;
				result.y = this->y * b.y;
				return result;
			}
			
			inline bool operator ==(const vector2<T> &v) const
			{
				return x==v.x && y==v.y;
			}
			
			inline bool operator !=(const vector2<T> &v) const
			{
				return x!=v.x || y!=v.y;
			}
		/*向量与向量、标量基本运算符重载*/
		
		//计算向量平方
		inline T get_square() const
		{
			return this->x*this->x + this->y*this->y;
		}
			
		inline void constrain( const T max_length );
		inline void normalize();
};

/*浮点向量等号运算特例*/
	template<>
	inline bool vector2<float>::operator ==(const vector2<float> &v) const
	{
			return (is_equal(x,v.x) && is_equal(y,v.y));
	}
	template<>
	inline bool vector2<double>::operator ==(const vector2<double> &v) const
	{
			return (is_equal(x,v.x) && is_equal(y,v.y));
	}
	
	template<>
	inline bool vector2<float>::operator !=(const vector2<float> &v) const
	{
			return (!is_equal(x,v.x) || !is_equal(y,v.y));
	}
	template<>
	inline bool vector2<double>::operator !=(const vector2<double> &v) const
	{
			return (!is_equal(x,v.x) || !is_equal(y,v.y));
	}
/*浮点向量等号运算特例*/

/*浮点向量限幅特例*/
	template<>
	inline void vector2<float>::constrain ( float max_length )
	{
		float length = get_square();
		float sq_max_length = max_length * max_length;
		if( length > sq_max_length )
		{
			float scale = sqrtf( sq_max_length / length );
			(*this) *= scale;
		}	
	}	
	template<>
	inline void vector2<double>::constrain ( double max_length )
	{
		double length = get_square();
		double sq_max_length = max_length * max_length;
		if( length > sq_max_length )
		{
			double scale = sqrt( sq_max_length / length );
			(*this) *= scale;
		}	
	}
/*浮点向量限幅特例*/
	
/*浮点向量归一化特例*/
	template<>
	inline void vector2<float>::normalize()
	{
		float length = safe_sqrt( get_square() );
		if( is_zero( length ) )
			return;
		
		length = 1.0f / length;
		*this *= length;
	}
	template<>
	inline void vector2<double>::normalize()
	{
		double length = safe_sqrt( get_square() );
		if( is_zero( length ) )
			return;
		
		length = 1.0 / length;
		*this *= length;
	}
/*浮点向量归一化特例*/