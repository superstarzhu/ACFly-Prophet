#include "M12_AccCalib.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "drv_Sensors.hpp"

//加速度计六面校准
//使用六个点算出椭球方程
//不需要完全放平

//朱文杰 20181226
//请勿用于商业用途
//！！抄袭必究！！

/*
	Ellipsoid equation :
		A( x - a )^2 + B( y - b )^2 + ( c - z )^2 - r = 0;

	es = [ a ]
			 [ b ]
			 [ c ]
			 [ A ]
			 [ B ]
			 [ r ]
	p = [ x1 y1 z1 ]
			[ x2 y2 z2 ]
			[ x3 y3 z3 ]
			[ x4 y4 z4 ]
			[ x5 y5 z5 ]
			[ x6 y6 z6 ]
*/

M12_AccCalib::M12_AccCalib():Mode_Base( "AccCalib", 12 )
{
	
}

static void get_F_dF( double F_matrix[6], double dF_matrix[36], double* es,  double* p );

#define Stable_Time 100
#define Calibration_Time 2000

ModeResult M12_AccCalib::main_func( void* param1, uint32_t param2 )
{	
	double inv_Calibration_Time = 1.0 / Calibration_Time;
	
	//牛顿迭代 雅可比矩阵
	double dF_matrix[ 6 * 6 ];
	double F_matrix[ 6 * 1 ];
	double dx_matrix[ 6 * 1 ];
	
	//判断板子是否静止
	vector3<double> Calibration_Acc_Max , Calibration_Acc_Min;
	vector3<double> Calibration_Gyro_Max , Calibration_Gyro_Min;
	//校准计数器
	int16_t Calibration_Counter;
	//已校准了的平面数（一共6面）
	uint8_t calibrated_surface_count;
	//记录已校准的平面
	bool calibrated_surface[6];
	//校准累加器
	vector3<int32_t> Acc_Sum[IMU_Sensors_Count] , Gyro_Sum[IMU_Sensors_Count];
	vector3<int32_t> Gyro_PSum[IMU_Sensors_Count];

	//校准结果
	double es[ 6 * 1 ];
	//加速度点
	double p[ IMU_Sensors_Count ][ 6 * 3 ];
	
	//初始化变量
	Calibration_Acc_Max.x = Calibration_Acc_Max.y = Calibration_Acc_Max.z = 0;	
	Calibration_Acc_Min.x = Calibration_Acc_Min.y = Calibration_Acc_Min.z = 0;
	Calibration_Gyro_Max.x = Calibration_Gyro_Max.y = Calibration_Gyro_Max.z = 0;	
	Calibration_Gyro_Min.x = Calibration_Gyro_Min.y = Calibration_Gyro_Min.z = 0;
	for( uint8_t i = 0; i < IMU_Sensors_Count; ++ i )
	{
		Acc_Sum[i].x = Acc_Sum[i].y = Acc_Sum[i].z = 0;
		Gyro_Sum[i].x = Gyro_Sum[i].y = Gyro_Sum[i].z = 0;
		Gyro_PSum[i].x = Gyro_PSum[i].y = Gyro_PSum[i].z = 0;
	}
	Calibration_Counter = -Stable_Time;
	calibrated_surface_count = 0;
	calibrated_surface[0] = calibrated_surface[1] = calibrated_surface[2] = calibrated_surface[3] = calibrated_surface[4] = calibrated_surface[5] = false;
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, 1 );
		if( calibrated_surface_count < 6 )
		{
			IMU_Sensor Acc_Sensor;
			IMU_Sensor Gyr_Sensor;
		  GetAccelerometer(0,&Acc_Sensor);
		  GetGyroscope(0,&Gyr_Sensor);
			vector3<int32_t> acc_data_raw = Acc_Sensor.data_raw;
		  vector3<int32_t> gyro_data_raw = Gyr_Sensor.data_raw;
			
			vector3<float> acc_f32( acc_data_raw.x, acc_data_raw.y, acc_data_raw.z );
		  acc_f32 = acc_f32*Acc_Sensor.sensitivity;
			
			vector3<float> gyro_f32( gyro_data_raw.x ,gyro_data_raw.y ,gyro_data_raw.z );
		  gyro_f32 = gyro_f32*Gyr_Sensor.sensitivity;
			
			//判断飞控哪面向上
			signed char Current_Surface = -1;
			if( acc_f32.x > 600.0f ) Current_Surface = 0;
			else if( acc_f32.x < -600.0f ) Current_Surface = 1;
			else if( acc_f32.y > 600.0f ) Current_Surface = 2;
			else if( acc_f32.y < -600.0f ) Current_Surface = 3;
			else if( acc_f32.z > 600.0f ) Current_Surface = 4;
			else if( acc_f32.z < -600.0f ) Current_Surface = 5;
			if( Current_Surface==-1 || calibrated_surface[Current_Surface]==true )
		  {	//当前面已校准或没放平
				setLedManualCtrl( 100, 0, 0, false, 0 );
				Calibration_Counter = -Stable_Time;				
				for( uint8_t i = 0 ; i < IMU_Sensors_Count ; ++ i )
				{
					Acc_Sum[i].x = Acc_Sum[i].y = Acc_Sum[i].z = 0;
					Gyro_Sum[i].x = Gyro_Sum[i].y = Gyro_Sum[i].z = 0;
				}			
			  continue;
		  }
			else
		  {
				vector3<double> acc_filted;
				vector3<double> gyro_filted;
		 	  get_AngularRate_Ctrl(&gyro_filted);
			  get_AccelerationENU_Ctrl(&acc_filted);
				
				/*判断飞控静止*/
					if( acc_filted.x > Calibration_Acc_Max.x ) Calibration_Acc_Max.x = acc_filted.x;
					else if( acc_filted.x < Calibration_Acc_Min.x ) Calibration_Acc_Min.x = acc_filted.x;
					if( acc_filted.y > Calibration_Acc_Max.y ) Calibration_Acc_Max.y = acc_filted.y;
					else if( acc_filted.y < Calibration_Acc_Min.y ) Calibration_Acc_Min.y = acc_filted.y;
					if( acc_filted.z > Calibration_Acc_Max.z ) Calibration_Acc_Max.z = acc_filted.z;
					else if( acc_filted.z < Calibration_Acc_Min.z ) Calibration_Acc_Min.z = acc_filted.z;
					
					if( gyro_filted.x > Calibration_Gyro_Max.x ) Calibration_Gyro_Max.x = gyro_filted.x;
					else if( gyro_filted.x < Calibration_Gyro_Min.x ) Calibration_Gyro_Min.x = gyro_filted.x;
					if( gyro_filted.y > Calibration_Gyro_Max.y )Calibration_Gyro_Max.y = gyro_filted.y;
					else if( gyro_filted.y < Calibration_Gyro_Min.y ) Calibration_Gyro_Min.y = gyro_filted.y;
					if( gyro_filted.z > Calibration_Gyro_Max.z ) Calibration_Gyro_Max.z = gyro_filted.z;
					else if( gyro_filted.z < Calibration_Gyro_Min.z ) Calibration_Gyro_Min.z = gyro_filted.z;

					double acc_fluctuation_range;	double gyro_fluctuation_range;
					vector3<double> v2=Calibration_Acc_Max-Calibration_Acc_Min;
					vector3<double> v1=Calibration_Gyro_Max-Calibration_Gyro_Min;
				 
					gyro_fluctuation_range=safe_sqrt(v1.get_square());
					acc_fluctuation_range=safe_sqrt(v2.get_square());
				 
					if( ( acc_fluctuation_range > 10 ) || ( gyro_fluctuation_range > 0.03 ) )
					{
						//板子不是静止的
						setLedManualCtrl( 100, 0, 0, false, 0 );
						Calibration_Counter = -Stable_Time;
						Calibration_Acc_Max = Calibration_Acc_Min = acc_filted;
						Calibration_Gyro_Max = Calibration_Gyro_Min = gyro_filted;
						for( unsigned char i = 0 ; i < IMU_Sensors_Count ; ++ i )
						{
							Acc_Sum[i].x = Acc_Sum[i].y = Acc_Sum[i].z = 0;
							Gyro_Sum[i].x = Gyro_Sum[i].y = Gyro_Sum[i].z = 0;
						}
						continue;
					}
				/*判断飞控静止*/
					
				if( Calibration_Counter < 0 )
				{
					for( uint8_t i = 0; i < IMU_Sensors_Count; ++ i )
					{
						Acc_Sum[i].x = Acc_Sum[i].y = Acc_Sum[i].z = 0;
						Gyro_Sum[i].x = Gyro_Sum[i].y = Gyro_Sum[i].z = 0;
					}
					++Calibration_Counter;
					setLedManualCtrl( 0, 0, 30, false, 0);
				}
				else
				{
					//采集数据点									
					for( uint8_t i = 0; i < IMU_Sensors_Count; ++i )
					{
						IMU_Sensor sensor;		
						if(GetAccelerometer(i,&sensor))
							Acc_Sum[i].set_vector(
								Acc_Sum[i].x + sensor.data_rawTC.x,
								Acc_Sum[i].y + sensor.data_rawTC.y,
								Acc_Sum[i].z + sensor.data_rawTC.z);
						if( GetGyroscope(i,&sensor))
							Gyro_Sum[i] = Gyro_Sum[i] + sensor.data_raw ;		
					}
					++Calibration_Counter;
					setLedManualCtrl( 0, 0, Calibration_Counter * 100 / Calibration_Time, false, 0 );
					
					if( Calibration_Counter == Calibration_Time )
					{
						//当前点已采集完成
						sendLedSignal(LEDSignal_Continue1);
						//记录点
						for( uint8_t i = 0; i < IMU_Sensors_Count; ++ i )
						{
							p[ i ][ Current_Surface * 3 ] = Acc_Sum[ i ].x * inv_Calibration_Time;
							p[ i ][ Current_Surface * 3 + 1 ] = Acc_Sum[ i ].y * inv_Calibration_Time;
							p[ i ][ Current_Surface * 3 + 2 ] = Acc_Sum[ i ].z * inv_Calibration_Time;						
							Gyro_PSum[i] = Gyro_PSum[i] + Gyro_Sum[i];
						}
						++calibrated_surface_count;
						calibrated_surface[ Current_Surface ] = true;		
						
						Calibration_Counter = -Stable_Time;
						for( uint8_t i = 0; i < IMU_Sensors_Count; ++ i )
						{
							Acc_Sum[i].x = Acc_Sum[i].y = Acc_Sum[i].z = 0;
							Gyro_Sum[i].x = Gyro_Sum[i].y = Gyro_Sum[i].z = 0;
						}
						continue;
					}
				}
		  }
		}
		else
			break;
  }

	//全部点都采集完成
	setLedMode(LEDMode_Processing1);
	PR_RESULT res = PR_OK;
	for( uint8_t i = 0; i < IMU_Sensors_Count; ++i )
	{
		IMU_Sensor sensor;
		if( GetAccelerometer( i, &sensor, 1 ) == true )
		{
			//初始化迭代起点
			es[0] = es[1] = es[2] = 0;
			es[3] = es[4] = 1;	
			es[5] = GravityAcc / sensor.sensitivity;
			es[5] *= es[5];
			Calibration_Counter = 0;
			
			while(1)
			{
				get_F_dF( F_matrix, dF_matrix, es, p[i] );
				bool status;
				status = Matrix_Inverse( dF_matrix , 6 );
				dx_matrix[0] = dF_matrix[0]*F_matrix[0] + dF_matrix[1]*F_matrix[1] + dF_matrix[2]*F_matrix[2] + dF_matrix[3]*F_matrix[3] + dF_matrix[4]*F_matrix[4] + dF_matrix[5]*F_matrix[5];
				dx_matrix[1] = dF_matrix[6]*F_matrix[0] + dF_matrix[7]*F_matrix[1] + dF_matrix[8]*F_matrix[2] + dF_matrix[9]*F_matrix[3] + dF_matrix[10]*F_matrix[4] + dF_matrix[11]*F_matrix[5];
				dx_matrix[2] = dF_matrix[12]*F_matrix[0] + dF_matrix[13]*F_matrix[1] + dF_matrix[14]*F_matrix[2] + dF_matrix[15]*F_matrix[3] + dF_matrix[16]*F_matrix[4] + dF_matrix[17]*F_matrix[5];
				dx_matrix[3] = dF_matrix[18]*F_matrix[0] + dF_matrix[19]*F_matrix[1] + dF_matrix[20]*F_matrix[2] + dF_matrix[21]*F_matrix[3] + dF_matrix[22]*F_matrix[4] + dF_matrix[23]*F_matrix[5];
				dx_matrix[4] = dF_matrix[24]*F_matrix[0] + dF_matrix[25]*F_matrix[1] + dF_matrix[26]*F_matrix[2] + dF_matrix[27]*F_matrix[3] + dF_matrix[28]*F_matrix[4] + dF_matrix[29]*F_matrix[5];
				dx_matrix[5] = dF_matrix[30]*F_matrix[0] + dF_matrix[31]*F_matrix[1] + dF_matrix[32]*F_matrix[2] + dF_matrix[33]*F_matrix[3] + dF_matrix[34]*F_matrix[4] + dF_matrix[35]*F_matrix[5];
							
				es[0] -= dx_matrix[0];
				es[1] -= dx_matrix[1];
				es[2] -= dx_matrix[2];
				es[3] -= dx_matrix[3];
				es[4] -= dx_matrix[4];
				es[5] -= dx_matrix[5];
				++Calibration_Counter;
				if( Calibration_Counter > 2000 && \
					( dx_matrix[0]*dx_matrix[0]\
					+ dx_matrix[1]*dx_matrix[1]\
					+ dx_matrix[2]*dx_matrix[2]\
					+ dx_matrix[3]*dx_matrix[3]\
					+ dx_matrix[4]*dx_matrix[4]
					+ dx_matrix[5]*dx_matrix[5]) < 0.01 )
				{	//精度达到 迭代结束
					es[3] = safe_sqrt(es[5]/es[3]);
					es[4] = safe_sqrt(es[5]/es[4]);
					es[5] = safe_sqrt(es[5]);
					double Gravity_DP = GravityAcc / sensor.sensitivity;
					IMUConfig cfg;
					cfg.STTemperature = 0;
					cfg.TemperatureCoefficient[0] = cfg.TemperatureCoefficient[1] = cfg.TemperatureCoefficient[2] = 0;
					res = ReadParamGroup( sensor.name+"_Acc", (uint64_t*)&cfg, 0 );					
					cfg.offset[0] = es[0];	cfg.offset[1] = es[1];	cfg.offset[2] = es[2];
					cfg.scale[0] = Gravity_DP / es[3];	cfg.scale[1] = Gravity_DP / es[4];	cfg.scale[2] = Gravity_DP / es[5];
					if( res == PR_OK )
						res = UpdateParamGroup( sensor.name+"_Acc", (uint64_t*)&cfg, 0, IMUConfigLength );
					else
						UpdateParamGroup( sensor.name+"_Acc", (uint64_t*)&cfg, 0, IMUConfigLength );
					break;
				}
				else if( Calibration_Counter > 5000 )
				{
					res = PR_ERR;
					break;
				}
			}
		}
	}
	if( res == PR_OK )
		sendLedSignal(LEDSignal_Success1);
	else
		sendLedSignal(LEDSignal_Err1);
	return MR_OK;
}

//get dF from matrix es[6,1] and matrix p[6,6]
static void get_F_dF( double F_matrix[6], double dF_matrix[36], double* es,  double* p )
{
	double a_x1 = es[0] - p[0];
	double a_x2 = es[0] - p[3];
	double a_x3 = es[0] - p[6];
	double a_x4 = es[0] - p[9];
	double a_x5 = es[0] - p[12];
	double a_x6 = es[0] - p[15];
	
	double b_y1 = es[1] - p[1];
	double b_y2 = es[1] - p[4];
	double b_y3 = es[1] - p[7];
	double b_y4 = es[1] - p[10];
	double b_y5 = es[1] - p[13];
	double b_y6 = es[1] - p[16];
	
	double c_z1 = es[2] - p[2];
	double c_z2 = es[2] - p[5];
	double c_z3 = es[2] - p[8];
	double c_z4 = es[2] - p[11];
	double c_z5 = es[2] - p[14];
	double c_z6 = es[2] - p[17];
	
	F_matrix[0] = es[3]*a_x1*a_x1 + es[4]*b_y1*b_y1 + c_z1*c_z1 - es[5];
	F_matrix[1] = es[3]*a_x2*a_x2 + es[4]*b_y2*b_y2 + c_z2*c_z2 - es[5];
	F_matrix[2] = es[3]*a_x3*a_x3 + es[4]*b_y3*b_y3 + c_z3*c_z3 - es[5];
	F_matrix[3] = es[3]*a_x4*a_x4 + es[4]*b_y4*b_y4 + c_z4*c_z4 - es[5];
	F_matrix[4] = es[3]*a_x5*a_x5 + es[4]*b_y5*b_y5 + c_z5*c_z5 - es[5];
	F_matrix[5] = es[3]*a_x6*a_x6 + es[4]*b_y6*b_y6 + c_z6*c_z6 - es[5];
	
	double A2  = es[3] * 2.0;
	double B2  = es[4] * 2.0;
	dF_matrix[00] = A2 * a_x1;	dF_matrix[01] = B2 * b_y1;	dF_matrix[02] = 2 * c_z1;	dF_matrix[03] = a_x1 * a_x1;	dF_matrix[04] = b_y1 * b_y1;	dF_matrix[05] = -1;
	dF_matrix[06] = A2 * a_x2;	dF_matrix[07] = B2 * b_y2;	dF_matrix[8] = 2 * c_z2;	dF_matrix[9] = a_x2 * a_x2;	  dF_matrix[10] = b_y2 * b_y2;	dF_matrix[11] = -1;
	dF_matrix[12] = A2 * a_x3;	dF_matrix[13] = B2 * b_y3;	dF_matrix[14] = 2 * c_z3;	dF_matrix[15] = a_x3 * a_x3;	dF_matrix[16] = b_y3 * b_y3;	dF_matrix[17] = -1;
	dF_matrix[18] = A2 * a_x4;	dF_matrix[19] = B2 * b_y4;	dF_matrix[20] = 2 * c_z4;	dF_matrix[21] = a_x4 * a_x4;	dF_matrix[22] = b_y4 * b_y4;	dF_matrix[23] = -1;
	dF_matrix[24] = A2 * a_x5;	dF_matrix[25] = B2 * b_y5;	dF_matrix[26] = 2 * c_z5;	dF_matrix[27] = a_x5 * a_x5;	dF_matrix[28] = b_y5 * b_y5;	dF_matrix[29] = -1;
	dF_matrix[30] = A2 * a_x6;	dF_matrix[31] = B2 * b_y6;	dF_matrix[32] = 2 * c_z6;	dF_matrix[33] = a_x6 * a_x6;	dF_matrix[34] = b_y6 * b_y6;	dF_matrix[35] = -1;
}
