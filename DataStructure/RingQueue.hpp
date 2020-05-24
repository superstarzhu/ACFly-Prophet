#ifndef RINGQUEUE
#define RINGQUEUE

#include<string.h>

//class AC_Link_Generater;

template < typename T >
class RingQueue
{
	private:
		T* datas;
		unsigned int capacity;
	
		bool overflow;
		volatile unsigned int current_pos , read_pos;
	public:
		inline void reset( T reset_value )
		{
			for( int i = 0 ; i < capacity ; ++i )
			{
				datas[i] = reset_value;
			}
		}
	
		RingQueue( unsigned int capacity )
		{
			datas = new T[ capacity ];
			this->capacity = capacity;
			
			overflow = false;
			current_pos = read_pos = 0;
		}
		~RingQueue()
		{
			delete[] datas;
		}
				
		inline unsigned int get_size()
		{
			if( read_pos <= current_pos ) return current_pos - read_pos;
			else return capacity + current_pos - read_pos;
		}
		inline unsigned int get_free_size()
		{
			if( read_pos > current_pos ) return read_pos - current_pos;
			else return capacity + read_pos - current_pos;
		}
		inline bool empty()
		{
			return( current_pos == read_pos );
		}
		inline void clear_overflow()
		{
			overflow = false;
		}
		inline T* get_member( unsigned int index )
		{
			if( index >= get_size() )return 0;
			if( index <= current_pos ) return &datas[ current_pos - index ];
			else return &datas[ current_pos + capacity - index ];
		}
		inline T* get_member_ring( unsigned int index )
		{
			++index;
			if( index > capacity )
				index = capacity;
			if( index <= current_pos ) return &datas[ current_pos - index ];
			else return &datas[ current_pos + capacity - index ];
		}
		
		inline bool align4()
		{
			if( current_pos & 3 )
			{
				current_pos &= ~3;
				current_pos += 4;
				if( current_pos >= capacity ) 
					current_pos -= capacity;
				if( current_pos == read_pos ) overflow = true;				
			}
			return overflow;
		}
		inline bool push( T new_data )
		{
			datas[current_pos] = new_data;
			++current_pos;
			if( current_pos == capacity ) current_pos = 0;
			if( current_pos == read_pos ) overflow = true;
			return overflow;
		}
		inline T* get_current()
		{
			return &datas[current_pos];
		}
		inline bool set_new()
		{
			++current_pos;
			if( current_pos == capacity ) current_pos = 0;
			if( current_pos == read_pos ) overflow = true;
			return overflow;
		}
		
		inline T pop()
		{
			if( !empty() )
			{
				unsigned int rd_pos = read_pos;
				if( ++read_pos == capacity ) read_pos = 0;
				return datas[rd_pos];
			}
			else return datas[read_pos];
		}
		inline T front()
		{
			return datas[read_pos];
		}
		inline T back()
		{
			if( current_pos == 0 ) return datas[capacity - 1];
			return datas[current_pos-1];
		}
		
		inline void clear()
		{
			read_pos = current_pos;
			overflow = false;
		}
		
		inline bool pop_dma_buf( T*& buf , unsigned int& length )	//return true if all data is pop
		{
			if( empty() )
			{
				length = 0;
				return true;
			}
			buf = &datas[read_pos];
			if( (current_pos >= read_pos) || (current_pos == 0) )
			{
				//looks like:
				//  |||read_pos|->|current_pos||||
				//read from read_pos to current_pos
				if( current_pos == 0 ) length = capacity - read_pos;
				else length = current_pos - read_pos;
				read_pos = current_pos;
				return true;
			}
			else
			{
				//looks like:
				//  |||current_pos||||read_pos|->|
				//read from read_pos to end
				length = capacity - read_pos;
				read_pos = 0;
				return false;
			}
		}
		
		inline bool copy_from( const T* origin , const unsigned int length )
		{
			unsigned int free_size = get_free_size();
			if( free_size < length ) return false;
			
			//get available buffer size from current_pos to the end of the array
			free_size = capacity - current_pos;
			if( free_size >= length )
			{
				memcpy( &this->datas[current_pos] , origin , length );
				current_pos += length;
				if( current_pos == capacity )
					current_pos = 0;
			}
			else
			{
				if( free_size > 0 ) memcpy( &this->datas[current_pos] , origin , free_size );
				memcpy( this->datas , &origin[free_size] , length - free_size );
				current_pos = length - free_size;
			}
			
			return true;
		}
		
		//bool copy_from( const AC_Link_Generater& cp_data );
};

#endif
