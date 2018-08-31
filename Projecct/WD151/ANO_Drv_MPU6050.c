
#include "i2c_soft.h"
#include "ANO_Drv_MPU6050.h"
#include "MPU6050.h"
//#include "bsp_SysTick.h"
#include "filter.h"
//#include "delay.h"
//#include "ANO_Config.h"
//#include "ANO_Param.h"
//#include "ANO_Data_Transfer.h"

_sensor_st sensor;

/*
传感器默认方向
     +x
     |
 +y--|--
     |

0:默认  
1：传感器顺时针90 度
2：传感器顺时针180度
3：传感器顺时针270度
*/

void sensor_dir(u8 dir,float itx,float ity,float itz,float *it_x,float *it_y,float *it_z)
{
	switch(dir)
	{
		case 1: //传感器顺时针90度
				*it_x = ity;
				*it_y = -itx;
				*it_z = itz;
		break;
		case 2://传感器顺时针180度
				*it_x = -itx;
				*it_y = -ity;
				*it_z = itz;
		break;
		case 3://传感器顺时针270度
				*it_x = -ity;
				*it_y = itx;
				*it_z = itz;
		break;
		default://传感器默认方向
			*it_x = itx;
			*it_y = ity;
			*it_z = itz;			
		break;
	}
	
}

#define MPU_WINDOW_NUM 5
#define MPU_STEEPEST_NUM 5

#define MPU_WINDOW_NUM_ACC 20
#define MPU_STEEPEST_NUM_ACC 20

_steepest_st steepest_ax;
_steepest_st steepest_ay;
_steepest_st steepest_az;
_steepest_st steepest_gx;
_steepest_st steepest_gy;
_steepest_st steepest_gz;

s32 steepest_ax_arr[MPU_WINDOW_NUM_ACC ];
s32 steepest_ay_arr[MPU_WINDOW_NUM_ACC ];
s32 steepest_az_arr[MPU_WINDOW_NUM_ACC ];
s32 steepest_gx_arr[MPU_WINDOW_NUM ];
s32 steepest_gy_arr[MPU_WINDOW_NUM ];
s32 steepest_gz_arr[MPU_WINDOW_NUM ];

//u32 test_time[5];
void MPU6050_Data_Prepare(float T)
{	
	float sensor_val[6];

	MPU6050_GetData();
	
	MPU6050_Offset();//校准函数

	/*读取buffer原始数据*/
	sensor.Acc_I16.x = ACC.X;
	sensor.Acc_I16.y = ACC.Y;
	sensor.Acc_I16.z = ACC.Z;
 
	sensor.Gyro_I16.x = (float) GYRO.X ;
	sensor.Gyro_I16.y = (float) GYRO.Y ;
	sensor.Gyro_I16.z = (float) GYRO.Z ;

//	sensor.Tempreature = ((((int16_t)mpu6050_buffer[6]) << 8) | mpu6050_buffer[7]); //tempreature
//	sensor.Ftempreature = sensor.Tempreature/340.0f + 36.5f;

	sensor_val[A_X] = sensor.Acc_I16.x ;
	sensor_val[A_Y] = sensor.Acc_I16.y ;
	sensor_val[A_Z] = sensor.Acc_I16.z;

	sensor_val[G_X] = sensor.Gyro_I16.x ;
	sensor_val[G_Y] = sensor.Gyro_I16.y ;
	sensor_val[G_Z] = sensor.Gyro_I16.z  ;	
	
//	test_time[0] = GetSysTime_us();
	/*梯度下降拟合数据*/	
	steepest_descend(steepest_ax_arr ,MPU_WINDOW_NUM_ACC ,&steepest_ax ,MPU_STEEPEST_NUM_ACC,(s32) sensor_val[A_X]);
	steepest_descend(steepest_ay_arr ,MPU_WINDOW_NUM_ACC ,&steepest_ay ,MPU_STEEPEST_NUM_ACC,(s32) sensor_val[A_Y]);
	steepest_descend(steepest_az_arr ,MPU_WINDOW_NUM_ACC ,&steepest_az ,MPU_STEEPEST_NUM_ACC,(s32) sensor_val[A_Z]);
	steepest_descend(steepest_gx_arr ,MPU_WINDOW_NUM ,&steepest_gx ,MPU_STEEPEST_NUM,(s32) sensor_val[G_X]);
	steepest_descend(steepest_gy_arr ,MPU_WINDOW_NUM ,&steepest_gy ,MPU_STEEPEST_NUM,(s32) sensor_val[G_Y]);
	steepest_descend(steepest_gz_arr ,MPU_WINDOW_NUM ,&steepest_gz ,MPU_STEEPEST_NUM,(s32) sensor_val[G_Z]);
//	test_time[1] = GetSysTime_us();
//	test_time[2] = test_time[1] - test_time[0];
	/*传感器方向调整*/
	sensor_dir( 2,													//加速度计方向
				(float)steepest_ax.now_out,
				(float)steepest_ay.now_out,//sensor_val[A_Z],//
				(float)steepest_az.now_out,
				&sensor.Acc.x ,
				&sensor.Acc.y ,
				&sensor.Acc.z );
						
	sensor_dir( 2,													//陀螺仪方向
				(float)steepest_gx.now_out,
				(float)steepest_gy.now_out,
				(float)steepest_gz.now_out,
				&sensor.Gyro.x, 
				&sensor.Gyro.y,
				&sensor.Gyro.z);
	
//======================================================================
	/*陀螺仪转换到度每秒*/
	sensor.Gyro_deg.x = sensor.Gyro.x *0.0610f ;//  /65535 * 4000; +-2000度
	sensor.Gyro_deg.y = sensor.Gyro.y *0.0610f ;
	sensor.Gyro_deg.z = sensor.Gyro.z *0.0610f ;
	Gyr_rad.X = -sensor.Gyro_deg.x;
	Gyr_rad.Y = -sensor.Gyro_deg.y;
	Gyr_rad.Z = sensor.Gyro_deg.z;
	/*加速度计转换到毫米每平方秒*/
	sensor.Acc_mmss.x = sensor.Acc.x *1.1962f ;//   /65535 * 16*9800; +-8G
	sensor.Acc_mmss.y = sensor.Acc.y *1.1962f ;
	sensor.Acc_mmss.z = sensor.Acc.z *1.1962f ;
}

