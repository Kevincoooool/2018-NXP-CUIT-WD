#include "control.h"
#include "mymath.h"
#include "Balance.h"
#include "ADRC.h"
#include "MPU6050.h"
#include "EM.h"
/******************************************************************************************************************
2018年全国大学生恩智浦杯智能车竞赛
成都信息工程大学   成信WD队
作者QQ：97354734
文件作用:速度检测
*******************************************************************************************************************/
uint8 Run_OK = 0;
uint8 Run_Flag = 0;
uint8 Stop_Flag = 0;
uint8 Run_Stop = 1;
uint8 Auto_Run = 0;
uint8 Fast_run = 0;
S_FLOAT_XYZ 
	GYRO_Real,		// 陀螺仪转化后的数据
	ACC_Real,		// 加速度计转化后的数据
	Attitude_Angle,	// 当前角度
	Last_Angle,		// 上次角度
	Target_Angle;	// 目标角度
	

S_INT16_XYZ
	GYRO,			// 陀螺仪原始数据
	GYRO_Offset,	// 陀螺仪温飘
	GYRO_Last,		// 陀螺仪上次数据
	ACC, 			// 加速度计数据
	ACC_Offset,		// 加速度计温飘
	ACC_Last;		// 加速度计上次数据
	
S_INT32_XYZ
	Tar_Ang_Vel,	// 目标角速度
	Tar_Ang_Vel_Last;	// 上次目标角速度

int32 
	Speed_Now = 0,	// 当前实际速度
	Speed_Min = 0,	// 左右最小速度
	Speed_Last = 0,	// 左右最小速度
	Speed_Set = 0, 	// 目标设定速度
	Speed_Diff = 0, 	// 目标设定速度
	Theory_Duty = 0,// 理论直立占空比
	Vel_Set = 0,	// 目标转向角速度
	Direct_Parameter = 0,// 转向系数
	Direct_Last = 0,
	Radius = 0;		// 目标转向半径倒数

uint8 Point = 80;
int32 Difference = 0;
int16 Diff_speed = 0;
_sensor_st sensor;
	  			/* 各种标志位，放定时器中进行时序控制 */
char Speed_Flag, Angle_Flag, Ang_Velocity_Flag;
extern _fix_inte_filter_st wcz_spe_fus;
char Left_Crazy = 0;	// 电机疯转
char Right_Crazy = 0;	// 电机疯转
int32 MOTOR_Duty_Left  = 0;
int32 MOTOR_Duty_Right = 0;
int MOTOR_Speed_Left = 0;
int MOTOR_Speed_Right = 0; 
int32 MOTOR_Speed_Left_Last = 0;
int32 MOTOR_Speed_Right_Last = 0;
int32 MOTOR_Left_Acc = 0;
int32 MOTOR_Right_Acc = 0;
volatile float  Distance = 0;
int32  Speed_Limit_Max =1200 ,Speed_Limit_Min = -800;

/******* 电机速度测量 ********/
uint16 temp1,temp2;
void Speed_Measure(void)
{
	static int32 Speed_Last = 0;
	static int32 Crazy_Count = 0;
	
	temp1 = ftm_count_get(ftm0);
	temp2 = ftm_count_get(ftm1);
	//计数器清零
	ftm_count_clean(ftm0);
	ftm_count_clean(ftm1);
	//根据方向信号判断正负，假设方向信号是高电平时为反转
	if(gpio_get(C5))    MOTOR_Speed_Right = -temp1;//速度取负 得到右轮转速
	else                MOTOR_Speed_Right = temp1;             
	if(gpio_get(H5))    MOTOR_Speed_Left 	= temp2;//速度取负 得到左轮转速
	else                MOTOR_Speed_Left 	= -temp2;
//	/******* 右电机速度相关控制 ********/

	MOTOR_Right_Acc = MOTOR_Speed_Right - MOTOR_Speed_Right_Last;	// 计算加速度
	if (MOTOR_Right_Acc > 100)
	{
		Right_Crazy = 1;	// 疯转
	}
	if (MOTOR_Speed_Right > Speed_Set + 200)
	{
		Right_Crazy = 2;	// 疯转
	}
	if (MOTOR_Speed_Right < -350)
	{
		Right_Crazy = -1;	// 倒转
	}
	
	if (Right_Crazy)
	{
		if (MOTOR_Right_Acc <= 100)
		{
			if (MOTOR_Speed_Right < Speed_Set + 200 && MOTOR_Speed_Right > 0)
			{
				Right_Crazy = 0;
			}
		}
	}
	
	if (!Right_Crazy)
	{
		MOTOR_Speed_Right = MOTOR_Speed_Right*0.9 + MOTOR_Speed_Right_Last*0.1;
		MOTOR_Speed_Right_Last = MOTOR_Speed_Right;	// 更新右轮速度
	}
	else
	{
		MOTOR_Speed_Right = MOTOR_Speed_Right*0.5 + MOTOR_Speed_Right_Last*0.5;
		MOTOR_Speed_Right_Last = MOTOR_Speed_Right;	// 更新右轮速度
	}
	/******* 右电机速度相关控制结束 ********/
	
	/******* 左电机速度相关控制 ********/
	
	MOTOR_Left_Acc = MOTOR_Speed_Left - MOTOR_Speed_Left_Last;	// 计算加速度
	if (MOTOR_Left_Acc > 100)
	{
		Left_Crazy = 1;
	}
	if (MOTOR_Speed_Left > Speed_Set + 200)
	{
		Left_Crazy = 2;
	}
	if (MOTOR_Speed_Left < -350)
	{
		Left_Crazy = -1;
	}
	
	if (Left_Crazy)
	{
		if (MOTOR_Left_Acc <= 100)
		{
			if (MOTOR_Speed_Left < Speed_Set + 200 && MOTOR_Speed_Left > 0)
			{
				Left_Crazy = 0;
			}
		}
	}
	
	if (!Left_Crazy)
	{
		MOTOR_Speed_Left = 0.9*MOTOR_Speed_Left + 0.1*MOTOR_Speed_Left_Last;	// 低通滤波
		MOTOR_Speed_Left_Last = MOTOR_Speed_Left;	// 更新左轮速度
	}
	else
	{
		MOTOR_Speed_Left = 0.5*MOTOR_Speed_Left + 0.5*MOTOR_Speed_Left_Last;	// 低通滤波
		MOTOR_Speed_Left_Last = MOTOR_Speed_Left;	// 更新左轮速度
	}

	
	
	/******* 左电机速度相关控制结束 ********/
	
	
	if ((Left_Crazy && Right_Crazy) || (Left_Crazy && MOTOR_Speed_Right < 10) || (Right_Crazy && MOTOR_Speed_Left < 10))
	{
		Crazy_Count++;
		if (Crazy_Count >= 200)
		{
			Crazy_Count = 0;
			Run_Flag = 0;
		}
	}
	else
	{
		Right_Crazy = 0;
	}
	
	/******* 电机疯转特殊处理 ********/
	if ((Left_Crazy > 0) && (Right_Crazy > 0))
	{
		Speed_Now = Speed_Last;			// 两边都疯转，使用上次速度作为当前实际速度
	}
	else if (Left_Crazy)
	{
		if (MOTOR_Speed_Right > Speed_Set)
		{
			Speed_Now = Speed_Last;
		}
		else
		{
			Speed_Now = MOTOR_Speed_Right;	// 左电机疯转，使用上次速度作为当前实际速度
		}
	}
	else if (Right_Crazy)
	{
		if (MOTOR_Speed_Left > Speed_Set)
		{
			Speed_Now = Speed_Last;
		}
		else
		{
			Speed_Now = MOTOR_Speed_Left;	// 右电机疯转，使用上次速度作为当前实际速度
		}
	}
	else
	{
		Speed_Now = (MOTOR_Speed_Left + MOTOR_Speed_Right) / 2;	// 左右取平均计算车子实际速度
	}
	
	Speed_Now = Speed_Now *0.8 + Speed_Last * 0.2;
	Speed_Last = Speed_Now;
	Fhan_ADRC(&ADRC_SPEED_MIN_Controller,Speed_Now);  //对当前速度进行TD过渡 
	Speed_Min = range_protect(ADRC_SPEED_MIN_Controller.x1,150,160);   //限幅越大转向越猛  且越不稳

}
_fix_inte_filter_st wcz_spe_fus;

s32 wcz_ref_speed;
float wcz_acc_deadzone;	
static s32 wcz_acc;
#define N_TIMES 5

void WCZ_Data_Calc(u8 dT_ms)//陀螺仪加速度积分速度和编码器速度融合
{

	wcz_ref_speed  = Speed_Now;
	wcz_acc        = -(s32)imu_data.w_acc.x/10;
	wcz_acc_deadzone = LIMIT(5 *(0.996f - imu_data.x_vec.x *imu_data.x_vec.x),0,1) *10;
	
	
	wcz_spe_fus.fix_kp = 0.2f;
	wcz_spe_fus.in_est_d = my_deadzone(wcz_acc,0,wcz_acc_deadzone);
	wcz_spe_fus.in_obs = wcz_ref_speed;
	wcz_spe_fus.e_limit = 100;
	fix_inte_filter(dT_ms*1e-3f,&wcz_spe_fus);

}
float battery=0;
void BAT_CHECK(void)
{
	battery = adc_once(ADC0_SE4,ADC_12bit)*3.3*12*10/2/4096;
	
}
void Speed_Control(void)
{
		switch(Mode_Set)
		{
		case 0:		Speed_Set = 0;// 默认初始速度
				 	break;
		case 1:		Speed_Set = 100;
				 	break;
		case 2:		Speed_Set = 120;	
				 	break;
		case 3:		Speed_Set = 150;
				 	break;
		case 4:		Speed_Set = 180;
				 	break;
		case 5:		Speed_Set = 210;
				 	break;
		case 6:		Speed_Set = 250;
					break;
		default:	Speed_Set = 0;
					break;
		}
}

void MOTOR_Control(int32 LDuty, int32 RDuty)
{
	if (LDuty > 0)
	{
		LDuty = range_protect(LDuty, -MOTOR_MAX, MOTOR_MAX);	// 限幅保护
		ftm_pwm_duty(ftm2,ftm_ch2,LDuty);	  	// 占空比最大990！！！
		ftm_pwm_duty(ftm2,ftm_ch3,0);	// 占空比最大990！！！
	}
	else
	{
		LDuty = range_protect(LDuty, -MOTOR_MAX, MOTOR_MAX);// 限幅保护
		ftm_pwm_duty(ftm2,ftm_ch2,0);	  	// 占空比最大990！！！
		ftm_pwm_duty(ftm2,ftm_ch3,-LDuty);	// 占空比最大990！！！
	}
	
	if (RDuty > 0)
	{
		RDuty = range_protect(RDuty, -MOTOR_MAX, MOTOR_MAX);	// 限幅保护
		ftm_pwm_duty(ftm2,ftm_ch0,0);	  	// 占空比最大990！！！
		ftm_pwm_duty(ftm2,ftm_ch1,RDuty);	// 占空比最大990！！！
	}
	else
	{
		RDuty = range_protect(RDuty, -MOTOR_MAX, MOTOR_MAX);// 限幅保护
		ftm_pwm_duty(ftm2,ftm_ch0,-RDuty);	  	// 占空比最大990！！！
		ftm_pwm_duty(ftm2,ftm_ch1,0);	// 占空比最大990！！！
	}
}

void Start_Control(void)
{
	if (Run_OK)
	{									
		if (Run_Stop)	// 由屏幕设置是否停车，默认停车
		{
			if (gpio_get(I5)==0)
			{
						
				Run_state = 4;
				Stop_Flag=1;
				Run_OK = 0;			//清除出发成功标志位
			}
		}			
	}
}
uint8 Fres = 0;	// 前瞻
void Speed_Calculate(void)//由当前速度决定方向PD的取值
{
	if (ADRC_SPEED_MIN_Controller.x1 < 110)
	{
		Fres = 0;
	}
	else if (ADRC_SPEED_MIN_Controller.x1 < 130)
	{
		Fres = 1;
	}
	else if (ADRC_SPEED_MIN_Controller.x1 < 150)
	{
		Fres = 2;
	}
	else if (ADRC_SPEED_MIN_Controller.x1 < 170)
	{
		Fres = 3;
	}
	else if (ADRC_SPEED_MIN_Controller.x1 >= 200)
	{
		Fres = 4;
	}
}
/******** 限幅保护 *********/
int32 range_protect(int32 duty, int32 min, int32 max)//限幅保护
{
	if (duty >= max)
	{
		return max;
	}
	if (duty <= min)
	{
		return min;
	}
	else
	{
		return duty;
	}
}
/* 初始化用到的一些变量 */
void Balance_Init(void)
{
	Attitude_Angle.Y = 0;
	Target_Angle.Y = 0;
	Tar_Ang_Vel.Y = 0;
	Tar_Ang_Vel.Z = 0;
	Speed_Min=100;
	Distance = 0;
	Ring_state = 0;
	Ring_Flag = 0;
	Run_OK = 0;
	Run_state = 0;
}

