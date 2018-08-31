#include "MPU6050.h"
#include "math.h"
#include "Balance.h"
#include <mymath.h>
#include "MyBalance.h"
#include "filter.h"
#include "PID.h"
#include "control.h"
#include "EM.h"
/******************************************************************************************************************
2018年全国大学生恩智浦杯智能车竞赛
成都信息工程大学   成信WD队
作者QQ：97354734
文件作用:MPU6050
*******************************************************************************************************************/
char Offset_OK = 0;
 //MPU6050测量的原始数据值
/*
 * 函数名：MPU6050_Init
 * 描述  ：陀螺仪初始化
 * 输入  ：无
 * 输出  ：0成功  1失败
 * 调用  ：外部调用
 */
uint8 MPU6050_Init(void)
{ 
	uint8 res;
	
	MPU6050_OPEN();           				// 初始化 mpu6050 接口
	systick_delay_ms(100);
	MPU6050_WR(MPU_PWR_MGMT1_REG,0X80);		// 复位MPU6050
  systick_delay_ms(5);
	MPU6050_WR(MPU_PWR_MGMT1_REG,0X00);		// 唤醒MPU6050 
	systick_delay_ms(5);
	MPU6050_WR(MPU_PWR_MGMT1_REG,0X01);		// 设置CLKSEL,PLL X轴为参考
	systick_delay_ms(5);
	MPU6050_WR(MPU_GYRO_CFG_REG,3<<3);		// 陀螺仪传感器,±2000dps	// 0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
	systick_delay_ms(5);
	MPU6050_WR(MPU_ACCEL_CFG_REG,2<<3);		// 加速度传感器,±8g		// 0,±2g;1,±4g;2,±8g;3,±16g
	systick_delay_ms(5);
	MPU6050_WR(MPU_SAMPLE_RATE_REG,0X00);	// 设置MPU6050的采样率,1KHz
	systick_delay_ms(5);
	MPU6050_WR(MPU_CFG_REG,0X04);			// 设置数字低通滤波器,20Hz
	systick_delay_ms(10);
	
	res=MPU6050_RD(MPU_DEVICE_ID_REG);
	if(res==MPU6050_ADRESS)// 器件ID正确
	{
		MPU6050_Offset();
		Balance_Init();
		return 0;
 	}
	else 
	{
		return 1;
	}
}

/*
 * 函数名：MPU6050_Offset
 * 描述  ：传感器采集零偏
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
void MPU6050_Offset(void)
{
	uint8 i, Count = 100;
	int64 temp[6] = {0};
	
	ACC_Offset.X = 0;
	ACC_Offset.Y = 0;
	ACC_Offset.Z = 0;
	GYRO_Offset.X = 0;
	GYRO_Offset.Y = 0;
	GYRO_Offset.Z = 0;
	
	for (i = 0; i < Count; i++)
	{
		MPU6050_GetData(&GYRO, &ACC);	// 读取陀螺仪数据
		systick_delay_ms(2);
		
		temp[0] += ACC.X;
		temp[1] += ACC.Y;
		temp[2] += ACC.Z;
		
		temp[3] += GYRO.X;
		temp[4] += GYRO.Y;
		temp[5] += GYRO.Z;
	}
	ACC_Offset.X = temp[0] / Count;
	ACC_Offset.Y = temp[1] / Count;
	ACC_Offset.Z = temp[2] / Count;
	
	GYRO_Offset.X = temp[3] / Count;
	GYRO_Offset.Y = temp[4] / Count;
	GYRO_Offset.Z = temp[5] / Count;
	
	Offset_OK = 1;
}
//角度计算与滤波
void Angle_Calculate(void)
{
	float Accel_Y;
	Accel_Y=fast_atan2(ACC_Real.X,ACC_Real.Z)*180/3.14159265f;                 //计算倾角	 
	Kalman_Filter(Accel_Y,-GYRO_Real.Y);            //调用卡尔曼滤波函数
	
}

/*
 * 函数名：MPU6050_GetData
 * 描述  ：获得传感器所有数据
 * 输入  ：*GYRO 陀螺仪		*ACC 加速度计
 * 输出  ：无
 * 调用  ：外部调用
 */
void MPU6050_GetData(S_INT16_XYZ *GYRO, S_INT16_XYZ *ACC)
{
	if (Offset_OK)
	{
		ACC->X = GetData(MPU_ACCEL_XOUTH_REG);	// 获取加速度计原始数据
		ACC->Y = GetData(MPU_ACCEL_YOUTH_REG);
		ACC->Z = GetData(MPU_ACCEL_ZOUTH_REG);
		
		GYRO->X = GetData(MPU_GYRO_XOUTH_REG) - GYRO_Offset.X;	// 获取陀螺仪原始数据
		GYRO->Y = GetData(MPU_GYRO_YOUTH_REG) - GYRO_Offset.Y;
		GYRO->Z = GetData(MPU_GYRO_ZOUTH_REG) - GYRO_Offset.Z;
	}
	else
	{
		ACC->X = GetData(MPU_ACCEL_XOUTH_REG);	// 获取加速度计原始数据并归一化
		ACC->Y = GetData(MPU_ACCEL_YOUTH_REG);
		ACC->Z = GetData(MPU_ACCEL_ZOUTH_REG);
		
		GYRO->X = GetData(MPU_GYRO_XOUTH_REG);	// 获取陀螺仪原始数据并归一化
		GYRO->Y = GetData(MPU_GYRO_YOUTH_REG);
		GYRO->Z = GetData(MPU_GYRO_ZOUTH_REG);
	}
}

/*
 * 函数名：GetData
 * 描述  ：获得16位数据
 * 输入  ：REG_Address 寄存器地址
 * 输出  ：返回寄存器数据
 * 调用  ：外部调用
 */
int16 GetData(uint8 REG_Address)
{
	uint8 H, L;

	H = MPU6050_RD(REG_Address);
	L = MPU6050_RD(REG_Address+1);
	
	return ((H<<8)|L);   //合成数据
}


#define AcceRatio 	16384.0f
#define GyroRatio 	16.4f
#define Gyro_Gr		0.0010653	// 角速度变成弧度	此参数对应陀螺2000度每秒
#define ACC_FILTER_NUM 5		// 加速度计滤波深度
#define GYRO_FILTER_NUM 5		// 陀螺仪滤波深度
int32 ACC_X_BUF[ACC_FILTER_NUM], ACC_Y_BUF[ACC_FILTER_NUM], ACC_Z_BUF[ACC_FILTER_NUM];	// 滤波缓存数组
int32 GYRO_X_BUF[GYRO_FILTER_NUM], GYRO_Y_BUF[GYRO_FILTER_NUM], GYRO_Z_BUF[GYRO_FILTER_NUM];
/*
 * 函数名：Data_Filter
 * 描述  ：数据滑动滤波
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void Data_Filter(void)	// 数据滤波
{
	uint8 i;
	int64 temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0, temp5 = 0, temp6 = 0;

	ACC_X_BUF[0] = ACC.X;	// 更新滑动窗口数组
	ACC_Y_BUF[0] = ACC.Y;
	ACC_Z_BUF[0] = ACC.Z;
	GYRO_X_BUF[0] = GYRO.X;
	GYRO_Y_BUF[0] = GYRO.Y;
	GYRO_Z_BUF[0] = GYRO.Z;

	for(i=0;i<ACC_FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
		
	}
	for(i=0;i<GYRO_FILTER_NUM;i++)
	{
		temp4 += GYRO_X_BUF[i];
		temp5 += GYRO_Y_BUF[i];
		temp6 += GYRO_Z_BUF[i];
	}
	
	ACC_Real.X = temp1 / ACC_FILTER_NUM;
	ACC_Real.Y = temp2 / ACC_FILTER_NUM;
	ACC_Real.Z = temp3 / ACC_FILTER_NUM;
	GYRO_Real.X = temp4 / GYRO_FILTER_NUM / GyroRatio ;
	GYRO_Real.Y = temp5 / GYRO_FILTER_NUM / GyroRatio ;
	GYRO_Real.Z = temp6 / GYRO_FILTER_NUM / GyroRatio;
	
	for(i = 0; i < ACC_FILTER_NUM - 1; i++)
	{
		ACC_X_BUF[ACC_FILTER_NUM-1-i] = ACC_X_BUF[ACC_FILTER_NUM-2-i];
		ACC_Y_BUF[ACC_FILTER_NUM-1-i] = ACC_Y_BUF[ACC_FILTER_NUM-2-i];
		ACC_Z_BUF[ACC_FILTER_NUM-1-i] = ACC_Z_BUF[ACC_FILTER_NUM-2-i];
		
	}
	for(i = 0; i < GYRO_FILTER_NUM - 1; i++)
	{
		GYRO_X_BUF[GYRO_FILTER_NUM-1-i] = GYRO_X_BUF[GYRO_FILTER_NUM-2-i];
		GYRO_Y_BUF[GYRO_FILTER_NUM-1-i] = GYRO_Y_BUF[GYRO_FILTER_NUM-2-i];
		GYRO_Z_BUF[GYRO_FILTER_NUM-1-i] = GYRO_Z_BUF[GYRO_FILTER_NUM-2-i];
	}
}
#define MPU_WINDOW_NUM 5
#define MPU_STEEPEST_NUM 5

#define MPU_WINDOW_NUM_ACC 15
#define MPU_STEEPEST_NUM_ACC 15

_steepest_st steepest_ax;
_steepest_st steepest_ay;
_steepest_st steepest_az;
_steepest_st steepest_gx;
_steepest_st steepest_gy;
_steepest_st steepest_gz;

int32 steepest_ax_arr[MPU_WINDOW_NUM_ACC ];
int32 steepest_ay_arr[MPU_WINDOW_NUM_ACC ];
int32 steepest_az_arr[MPU_WINDOW_NUM_ACC ];
int32 steepest_gx_arr[MPU_WINDOW_NUM ];
int32 steepest_gy_arr[MPU_WINDOW_NUM ];
int32 steepest_gz_arr[MPU_WINDOW_NUM ];

void Data_steepest(void)
{
	steepest_descend(steepest_ax_arr ,MPU_WINDOW_NUM_ACC ,&steepest_ax ,MPU_STEEPEST_NUM_ACC,(int32) ACC.X);
	steepest_descend(steepest_ay_arr ,MPU_WINDOW_NUM_ACC ,&steepest_ay ,MPU_STEEPEST_NUM_ACC,(int32) ACC.Y);
	steepest_descend(steepest_az_arr ,MPU_WINDOW_NUM_ACC ,&steepest_az ,MPU_STEEPEST_NUM_ACC,(int32) ACC.Z);
	steepest_descend(steepest_gx_arr ,MPU_WINDOW_NUM ,&steepest_gx ,MPU_STEEPEST_NUM,(int32) GYRO.X);
	steepest_descend(steepest_gy_arr ,MPU_WINDOW_NUM ,&steepest_gy ,MPU_STEEPEST_NUM,(int32) GYRO.Y);
	steepest_descend(steepest_gz_arr ,MPU_WINDOW_NUM ,&steepest_gz ,MPU_STEEPEST_NUM,(int32) GYRO.Z);


	sensor.Gyro_deg.x  = GYRO_Real.X = steepest_gx.now_out *0.0610f ;
	sensor.Gyro_deg.y  = GYRO_Real.Y = steepest_gy.now_out *0.0610f ;
	sensor.Gyro_deg.z  = GYRO_Real.Z = steepest_gz.now_out *0.0610f;
	sensor.Acc_mmss.x  = ACC_Real.X  = steepest_ax.now_out *2.3926f ;
	sensor.Acc_mmss.y  = ACC_Real.Y  = steepest_ay.now_out *2.3926f;
	sensor.Acc_mmss.z  = ACC_Real.Z  = steepest_az.now_out *2.3926f ;
	
}


_imu_st imu_data =  {1,0,0,0,
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					 0,0,0};
_lf_t err_lf_x;
_lf_t err_lf_y;
_lf_t err_lf_z;

_xyz_f_st vec_err_i;
	_xyz_f_st x_vec;
	_xyz_f_st y_vec;
	_xyz_f_st z_vec;
	_xyz_f_st a_acc;
	_xyz_f_st w_acc;
					 
void IMU_update(float dT,_xyz_f_st *gyr, _xyz_f_st *acc,_imu_st *imu)
{
	float kp = 0.7,ki = 0;
	
	float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q3q3,q1q2,q0q3;
	float w_q,x_q,y_q,z_q;
	float acc_length,q_length;
	_xyz_f_st acc_norm;
	_xyz_f_st vec_err;
	_xyz_f_st d_angle;
	  

    w_q = imu->w;
    x_q = imu->x;
    y_q = imu->y;
    z_q = imu->z;
	
//		q0q0 = w_q * w_q;							
		q0q1 = w_q * x_q;
		q0q2 = w_q * y_q;
		q1q1 = x_q * x_q;
		q1q3 = x_q * z_q;
		q2q2 = y_q * y_q;
		q2q3 = y_q * z_q;
		q3q3 = z_q * z_q;
		q1q2 = x_q * y_q;
		q0q3 = w_q * z_q;
	
    //
		
    // 加速度计的读数，单位化。
    acc_length = my_sqrt(my_pow(acc->x) + my_pow(acc->y) + my_pow(acc->z));
    acc_norm.x = acc->x / acc_length;
    acc_norm.y = acc->y / acc_length;
    acc_norm.z = acc->z / acc_length;

		
	// 载体坐标下的x方向向量，单位化。
    imu->x_vec.x = 1 - (2*q2q2 + 2*q3q3);
    imu->x_vec.y = 2*q1q2 - 2*q0q3;
    imu->x_vec.z = 2*q1q3 + 2*q0q2;
		
	// 载体坐标下的y方向向量，单位化。
    imu->y_vec.x = 2*q1q2 + 2*q0q3;
    imu->y_vec.y = 1 - (2*q1q1 + 2*q3q3);
    imu->y_vec.z = 2*q2q3 - 2*q0q1;
		
    // 载体坐标下的z方向向量（等效重力向量、重力加速度向量），单位化。
    imu->z_vec.x = 2*q1q3 - 2*q0q2;
    imu->z_vec.y = 2*q2q3 + 2*q0q1;
    imu->z_vec.z = 1 - (2*q1q1 + 2*q2q2);
		
	// 计算载体坐标下的运动加速度。(与姿态解算无关)
	imu->a_acc.x = acc->x - 9800 *imu->z_vec.x;
	imu->a_acc.y = acc->y - 9800 *imu->z_vec.y;
	imu->a_acc.z = acc->z - 9800 *imu->z_vec.z;
   
	// 计算世界坐标下的运动加速度。(与姿态解算无关)
	imu->w_acc.x = imu->x_vec.x *imu->a_acc.x + imu->x_vec.y *imu->a_acc.y + imu->x_vec.z *imu->a_acc.z;
	imu->w_acc.y = imu->y_vec.x *imu->a_acc.x + imu->y_vec.y *imu->a_acc.y + imu->y_vec.z *imu->a_acc.z;
	imu->w_acc.z = imu->z_vec.x *imu->a_acc.x + imu->z_vec.y *imu->a_acc.y + imu->z_vec.z *imu->a_acc.z;
    // 测量值与等效重力向量的叉积（计算向量误差）。
  vec_err.x =  (acc_norm.y * imu->z_vec.z - imu->z_vec.y * acc_norm.z);
  vec_err.y = -(acc_norm.x * imu->z_vec.z - imu->z_vec.x * acc_norm.z);
  vec_err.z = -(acc_norm.y * imu->z_vec.x - imu->z_vec.y * acc_norm.x);
		
	//截止频率1hz的低通限幅滤波
	limit_filter(dT,0.2f,&err_lf_x,vec_err.x);
	limit_filter(dT,0.2f,&err_lf_y,vec_err.y);
	limit_filter(dT,0.2f,&err_lf_z,vec_err.z);
	
	//误差积分
	vec_err_i.x += err_lf_x.out *dT *ki;
	vec_err_i.y += err_lf_y.out *dT *ki;
	vec_err_i.z += err_lf_z.out *dT *ki;
		
    // 构造增量旋转（含融合纠正）。
    d_angle.x = (gyr->x *RAD_PER_DEG + (err_lf_x.out + vec_err_i.x) * kp) * dT / 2 ;
    d_angle.y = (gyr->y *RAD_PER_DEG + (err_lf_y.out + vec_err_i.y) * kp) * dT / 2 ;
    d_angle.z = (gyr->z *RAD_PER_DEG + (err_lf_z.out + vec_err_i.z) * kp) * dT / 2 ;
    
    // 计算姿态。
    imu->w = w_q           - x_q*d_angle.x - y_q*d_angle.y - z_q*d_angle.z;
    imu->x = w_q*d_angle.x + x_q           + y_q*d_angle.z - z_q*d_angle.y;
    imu->y = w_q*d_angle.y - x_q*d_angle.z + y_q           + z_q*d_angle.x;
    imu->z = w_q*d_angle.z + x_q*d_angle.y - y_q*d_angle.x + z_q;
		
	q_length = my_sqrt(imu->w*imu->w + imu->x*imu->x + imu->y*imu->y + imu->z*imu->z);
    imu->w /= q_length;
    imu->x /= q_length;
    imu->y /= q_length;
    imu->z /= q_length;
	
		imu->pit = asin(2*q1q3 - 2*q0q2)*57.30f;
		imu->rol = fast_atan2(2*q2q3 + 2*q0q1, -2*q1q1-2*q2q2 + 1)*57.30f; 
		imu->yaw = -fast_atan2(2*q1q2 + 2*q0q3, -2*q2q2-2*q3q3 + 1)*57.30f; 
  
    
}
