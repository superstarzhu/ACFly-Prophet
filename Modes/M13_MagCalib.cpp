#include "M13_MagCalib.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"

//磁力计校准
//最小二乘椭球拟合
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
*/

M13_MagCalib::M13_MagCalib():Mode_Base( "MagCalib", 13 )
{
	
}

struct MagCalibInfo
{
	double sum_x[ IMU_Sensors_Count ];
	double sum_x2[ IMU_Sensors_Count ];
	double sum_x3[ IMU_Sensors_Count ];
	double sum_x4[ IMU_Sensors_Count ];
	double sum_y[ IMU_Sensors_Count ];
	double sum_y2[ IMU_Sensors_Count ];
	double sum_y3[ IMU_Sensors_Count ];
	double sum_y4[ IMU_Sensors_Count ];
	double sum_z[ IMU_Sensors_Count ];
	double sum_z2[ IMU_Sensors_Count ];
	double sum_z3[ IMU_Sensors_Count ];
	double sum_xy[ IMU_Sensors_Count ];
	double sum_x2y[ IMU_Sensors_Count ];
	double sum_xy2[ IMU_Sensors_Count ];
	double sum_x2y2[ IMU_Sensors_Count ];
	double sum_xz[ IMU_Sensors_Count ];
	double sum_x2z[ IMU_Sensors_Count ];
	double sum_xz2[ IMU_Sensors_Count ];
	double sum_x2z2[ IMU_Sensors_Count ];
	double sum_yz[ IMU_Sensors_Count ];
	double sum_y2z[ IMU_Sensors_Count ];
	double sum_yz2[ IMU_Sensors_Count ];
	double sum_y2z2[ IMU_Sensors_Count ];
	double n;
};
static void get_F_dF( uint8_t index, double es[6], double F_matrix[6], double dF_matrix[36], const MagCalibInfo* info );
ModeResult M13_MagCalib::main_func( void* param1, uint32_t param2 )
{	
	//牛顿迭代法 雅可比矩阵
	double dF_matrix[ 6 * 6 ];
	double F_matrix[ 6 * 1 ];
	double dx_matrix[ 6 * 1 ];

	//校准结果
	double es[ 6 * 1 ];
	
	//最小二乘的累加和
	MagCalibInfo info;
	
	//校准状态机
	TIME rotate_start_time;
	
	//记录最大最小磁力用于判断校准是否完成
	signed int max_x , min_x , max_y , min_y , max_z , min_z;
	bool sensor_present[ IMU_Sensors_Count ];
	
	//初始化
	max_x = min_x = max_y = min_y = max_z = min_z = 0;
	rotate_start_time.set_invalid();
	for( uint8_t i = 0; i < IMU_Sensors_Count; ++i )
	{
		info.sum_x[i] = info.sum_x2[i] = info.sum_x3[i] = info.sum_x4[i] = 0;
		info.sum_y[i] = info.sum_y2[i] = info.sum_y3[i] = info.sum_y4[i] = 0;
		info.sum_z[i] = info.sum_z2[i] = info.sum_z3[i] = 0;
		info.sum_xy[i] = info.sum_x2y[i] = info.sum_xy2[i] = info.sum_x2y2[i] = 0;
		info.sum_xz[i] = info.sum_x2z[i] = info.sum_xz2[i] = info.sum_x2z2[i] = 0;
		info.sum_yz[i] = info.sum_y2z[i] = info.sum_yz2[i] = info.sum_y2z2[i] = 0;
		info.n = 0;
		sensor_present[i] = true;
	}
	setLedManualCtrl( 100, 0, 0, false, 0 );
	
	while(1)
	{
		os_delay(0.02);
		
		vector3<double> angular_rate;
		get_AngularRate_Ctrl( &angular_rate, 1 );
		double rotate_speed = safe_sqrt( angular_rate.get_square() );
		if( rotate_speed > 0.6f )
		{
			if( rotate_start_time.is_valid() == false )
				rotate_start_time = TIME::now();
		}
		else
			rotate_start_time.set_invalid();
		if( rotate_start_time.is_valid() && rotate_start_time.get_pass_time()>0.2f )
		{
			//update min and max mag to check if calibration data is ready
			IMU_Sensor internal_magnetometer;
			GetMagnetometer( Internal_Magnetometer_Index, &internal_magnetometer, 1 );
			if( internal_magnetometer.data_raw.x > max_x )
				max_x = internal_magnetometer.data_raw.x;
			else if( internal_magnetometer.data_raw.x < min_x )
				min_x = internal_magnetometer.data_raw.x;
			if( internal_magnetometer.data_raw.y > max_y )
				max_y = internal_magnetometer.data_raw.y;
			else if( internal_magnetometer.data_raw.y < min_y )
				min_y = internal_magnetometer.data_raw.y;
			if( internal_magnetometer.data_raw.z > max_z )
				max_z = internal_magnetometer.data_raw.z;
			else if( internal_magnetometer.data_raw.z < min_z )
				min_z = internal_magnetometer.data_raw.z;
			
			/*update sums*/
				for( uint8_t i = 0; i < IMU_Sensors_Count; ++i )
				{
					IMU_Sensor sensor;							
					if( sensor_present[i] && GetMagnetometer( i, &sensor, 1 ) )
					{
						double x = sensor.data_raw.x;	double y = sensor.data_raw.y;	double z = sensor.data_raw.z;
						double x2 = x * x;	double x3 = x2 * x; double x4 = x3 * x;
						double y2 = y * y;	double y3 = y2 * y; double y4 = y3 * y;
						double z2 = z * z;	double z3 = z2 * z;
						double xy = x * y;	double x2y = x * xy;	double xy2 = xy * y;	double x2y2 = x * xy2;	
						double xz = x * z;	double x2z = x * xz;	double xz2 = xz * z;	double x2z2 = x * xz2;	
						double yz = y * z;	double y2z = y * yz;	double yz2 = yz * z;	double y2z2 = y * yz2;	
					
						info.sum_x[i] += x;	info.sum_x2[i] += x2;	info.sum_x3[i] += x3;	info.sum_x4[i] += x4;
						info.sum_y[i] += y;	info.sum_y2[i] += y2;	info.sum_y3[i] += y3;	info.sum_y4[i] += y4;
						info.sum_z[i] += z;	info.sum_z2[i] += z2;	info.sum_z3[i] += z3;
						info.sum_xy[i] += xy;	info.sum_x2y[i] += x2y;	info.sum_xy2[i] += xy2;	info.sum_x2y2[i] += x2y2;
						info.sum_xz[i] += xz;	info.sum_x2z[i] += x2z;	info.sum_xz2[i] += xz2;	info.sum_x2z2[i] += x2z2;
						info.sum_yz[i] += yz;	info.sum_y2z[i] += y2z;	info.sum_yz2[i] += yz2;	info.sum_y2z2[i] += y2z2;						
					}
					else
						sensor_present[i] = false;
				}
				++info.n;
			/*update sums*/
			
			setLedManualCtrl( 0, 0, (info.n>1500) ? 100 : info.n / 15, false, 0 );
			double calibrate_complete_condition = 0.4 / internal_magnetometer.sensitivity;
			if( 
				(max_x - min_x > calibrate_complete_condition) && \
				(max_y - min_y > calibrate_complete_condition) && \
				(max_z - min_z > calibrate_complete_condition) && \
				(info.n > 1500) 
			)
			{
				sendLedSignal(LEDSignal_Continue1);
				break;
			}
			else if( info.n > 5000 )
			{
				sendLedSignal(LEDSignal_Err1);
				return MR_Err;
			}
		}
		else
			setLedManualCtrl( 100, 0, 0, false, 0 );
	}
	
	//计算校准数据
	setLedMode(LEDMode_Processing1);
	PR_RESULT res = PR_OK;
	for( uint8_t i = 0; i < IMU_Sensors_Count; ++i )
	{
		IMU_Sensor sensor;
		if( sensor_present[i] && GetMagnetometer( i, &sensor, 1 ) )
		{
			//初始化校准
			uint16_t counter = 0;
			es[0] = es[1] = es[2] = 0;
			es[3] = es[4] = 1;	es[5] = 0.5f / sensor.sensitivity; es[5] *= es[5];
			while(1)
			{
				get_F_dF( i, es, F_matrix, dF_matrix, &info );
				
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
				++counter;
				
				if( counter > 1000 && ( dx_matrix[0]*dx_matrix[0] + dx_matrix[1]*dx_matrix[1] + dx_matrix[2]*dx_matrix[2] + dx_matrix[3]*dx_matrix[3] + dx_matrix[4]*dx_matrix[4] ) < 0.01 )
				{
					es[3] = safe_sqrt(es[3]);
					es[4] = safe_sqrt(es[4]);
					es[5] = safe_sqrt(es[5]);
					vector3<double> magnetic_force( es[5] / es[3] , es[5] / es[4] , es[5] );
					vector3<double> offset( es[0] , es[1] , es[2] );
					double standard_magnetic_force = 0.6 / sensor.sensitivity;
					vector3<double> scale = { standard_magnetic_force / magnetic_force.x , \
																		standard_magnetic_force / magnetic_force.y , \
																		standard_magnetic_force / magnetic_force.z };
					IMUConfig cfg;
					cfg.STTemperature = 0;
					cfg.TemperatureCoefficient[0] = cfg.TemperatureCoefficient[1] = cfg.TemperatureCoefficient[2] = 0;
					cfg.offset[0] = offset.x;	cfg.offset[1] = offset.y;	cfg.offset[2] = offset.z;
					cfg.scale[0] = scale.x;	cfg.scale[1] = scale.y;	cfg.scale[2] = scale.z;
					if( res == PR_OK )
						res = UpdateParamGroup( sensor.name+"_Mag", (uint64_t*)&cfg, 0, IMUConfigLength );
					else
						UpdateParamGroup( sensor.name+"_Mag", (uint64_t*)&cfg, 0, IMUConfigLength );
					break;
				}
				else if( counter > 5000 )
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

static void get_F_dF( uint8_t index, double es[6], double F_matrix[6], double dF_matrix[36], const MagCalibInfo* info )
{
	double Aa = es[3] * es[0];	double Bb = es[4] * es[1];
	double Aa2 = Aa*es[0];	double Bb2 = Bb*es[1];
	double Aa_2 = Aa * 2;	double Bb_2 = Bb * 2;
	double c2 = es[2]*es[2];	double c_2 = es[2] * 2;
	double A_2 = es[3] * 2;	double B_2 = es[4] * 2;
	double a2 = es[0]*es[0]; double a_2 = 2 * es[0];
	double b2 = es[1]*es[1];	double b_2 = 2 * es[1];
	 
	double temp1 = Aa2 + Bb2 + c2 - es[5]; 
	F_matrix[0] = es[3]*info->sum_x3[index]  - Aa_2*info->sum_x2[index] + es[4]*info->sum_xy2[index] - Bb_2*info->sum_xy[index] + info->sum_xz2[index] - c_2*info->sum_xz[index] + temp1*info->sum_x[index];
	F_matrix[1] = es[3]*info->sum_x2y[index] - Aa_2*info->sum_xy[index] + es[4]*info->sum_y3[index]  - Bb_2*info->sum_y2[index] + info->sum_yz2[index] - c_2*info->sum_yz[index] + temp1*info->sum_y[index];
	F_matrix[2] = es[3]*info->sum_x2z[index] - Aa_2*info->sum_xz[index] + es[4]*info->sum_y2z[index] - Bb_2*info->sum_yz[index] + info->sum_z3[index]  - c_2*info->sum_z2[index] + temp1*info->sum_z[index];
	F_matrix[3] = es[3]*info->sum_x4[index]   - Aa_2*info->sum_x3[index]  + es[4]*info->sum_x2y2[index] - Bb_2*info->sum_x2y[index] + info->sum_x2z2[index] - c_2*info->sum_x2z[index] + temp1*info->sum_x2[index];
	F_matrix[4] = es[3]*info->sum_x2y2[index] - Aa_2*info->sum_xy2[index] + es[4]*info->sum_y4[index]   - Bb_2*info->sum_y3[index]  + info->sum_y2z2[index] - c_2*info->sum_y2z[index] + temp1*info->sum_y2[index];
	F_matrix[5] = - Aa_2*info->sum_x[index] - Bb_2*info->sum_y[index] - c_2*info->sum_z[index] + es[3]*info->sum_x2[index] + es[4]*info->sum_y2[index] + info->sum_z2[index] + temp1*info->n;
	
	dF_matrix[0] = Aa_2*info->sum_x[index] - A_2*info->sum_x2[index];    dF_matrix[1] = Bb_2*info->sum_x[index] - B_2*info->sum_xy[index];    dF_matrix[2] = 2 * ( es[2]*info->sum_x[index] - info->sum_xz[index] );	  dF_matrix[3] = a2*info->sum_x[index] - a_2*info->sum_x2[index] + info->sum_x3[index];      dF_matrix[4] = b2*info->sum_x[index] - b_2*info->sum_xy[index] + info->sum_xy2[index];     dF_matrix[5]  = -info->sum_x[index];
	dF_matrix[6] = Aa_2*info->sum_y[index] - A_2*info->sum_xy[index];    dF_matrix[7] = Bb_2*info->sum_y[index] - B_2*info->sum_y2[index];    dF_matrix[8] = 2 * ( es[2]*info->sum_y[index] - info->sum_yz[index] );    dF_matrix[9] = a2*info->sum_y[index] - a_2*info->sum_xy[index] + info->sum_x2y[index];     dF_matrix[10] = b2*info->sum_y[index] - b_2*info->sum_y2[index] + info->sum_y3[index];     dF_matrix[11] = -info->sum_y[index];
	dF_matrix[12] = Aa_2*info->sum_z[index] - A_2*info->sum_xz[index];   dF_matrix[13] = Bb_2*info->sum_z[index] - B_2*info->sum_yz[index];   dF_matrix[14] = 2 * ( es[2]*info->sum_z[index] - info->sum_z2[index] );   dF_matrix[15] = a2*info->sum_z[index] - a_2*info->sum_xz[index] + info->sum_x2z[index];    dF_matrix[16] = b2*info->sum_z[index] - b_2*info->sum_yz[index] + info->sum_y2z[index];    dF_matrix[17] = -info->sum_z[index];
	dF_matrix[18] = Aa_2*info->sum_x2[index] - A_2*info->sum_x3[index];  dF_matrix[19] = Bb_2*info->sum_x2[index] - B_2*info->sum_x2y[index]; dF_matrix[20] = 2 * ( es[2]*info->sum_x2[index] - info->sum_x2z[index] ); dF_matrix[21] = a2*info->sum_x2[index] - a_2*info->sum_x3[index] + info->sum_x4[index];    dF_matrix[22] = b2*info->sum_x2[index] - b_2*info->sum_x2y[index] + info->sum_x2y2[index]; dF_matrix[23] = -info->sum_x2[index];
	dF_matrix[24] = Aa_2*info->sum_y2[index] - A_2*info->sum_xy2[index]; dF_matrix[25] = Bb_2*info->sum_y2[index] - B_2*info->sum_y3[index];  dF_matrix[26] = 2 * ( es[2]*info->sum_y2[index] - info->sum_y2z[index] ); dF_matrix[27] = a2*info->sum_y2[index] - a_2*info->sum_xy2[index] + info->sum_x2y2[index]; dF_matrix[28] = b2*info->sum_y2[index] - b_2*info->sum_y3[index] + info->sum_y4[index];    dF_matrix[29] = -info->sum_y2[index];
	dF_matrix[30] = Aa_2*info->n - A_2*info->sum_x[index];               dF_matrix[31] = Bb_2*info->n - B_2*info->sum_y[index];               dF_matrix[32] = 2 * ( es[2]*info->n - info->sum_z[index] );               dF_matrix[33] = a2*info->n - a_2*info->sum_x[index] + info->sum_x2[index];                 dF_matrix[34] = b2*info->n - b_2*info->sum_y[index] + info->sum_y2[index];                 dF_matrix[35] = -info->n;
}