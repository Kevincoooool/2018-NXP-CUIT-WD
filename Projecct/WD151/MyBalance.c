#include "MyBalance.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "System.h"
#include "MY_PID.h"


PID_arg_t arg_angle ;
PID_arg_t arg_gyro ;
PID_arg_t arg_speed ;
PID_arg_t arg_turn ;



PID_val_t val_angle;
PID_val_t val_gyro;
PID_val_t val_speed;
PID_val_t val_turn;

float ACC_Angle,Car_Angle,Angle_Speed;
int Acc_X,Acc_Y,Acc_Z,Gyro_X,Gyro_Y,Gyro_Z;

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
int Moto1,Moto2;

void My_PIDInit(void)
{
	arg_angle.kp = 40.0f;
	arg_angle.ki = 0.0f;
	arg_angle.kd = 0.1f ;
	arg_angle.k_pre_d = 0.0f;
	arg_angle.k_ff = 0.1f;
	
	arg_gyro.kp = 0.8f ;
	arg_gyro.ki = 0.0f ;
	arg_gyro.kd = 0.02f ;
	arg_gyro.k_pre_d = 0.0f;
	arg_gyro.k_ff = 0.0f;

	arg_speed.kp = 15.0f;
	arg_speed.ki = 0.0f ;
	arg_speed.kd = 0.01f ;
	arg_speed.k_pre_d = 0.0f;
	arg_speed.k_ff = 0.0f;		
	
	arg_turn.kp = 1.0f;
	arg_turn.ki = 0.0f ;
	arg_turn.kd = 0.0f ;
	arg_turn.k_pre_d = 0.0f;
	arg_turn.k_ff = 0.0f;		
}
void My_Control(void) 
{    
	MPU6050_GetData(&GYRO, &ACC);	// 读取陀螺仪数据
	Data_Filter();					// 对原始数据滑动滤波
	Angle_Calculate();
	Speed_Measure();
	
		PID_calculate(0.005,            			//周期
					0,						//前馈
					0,			//期望值（设定值）
					Car_Angle,			//反馈值
					&arg_angle, 			//PID参数结构体
					&val_angle,				//PID数据结构体
					1000,			//integral limit，积分限幅
					&Balance_Pwm);	//输出
	
		PID_calculate(0.005,            			//周期
					0,						//前馈
					0,			//期望值（设定值）
					Speed_Now,			//反馈值
					&arg_speed, 			//PID参数结构体
					&val_speed,				//PID数据结构体
					1000,			//integral limit，积分限幅
					&Velocity_Pwm  );	//输出

 	Moto1=Balance_Pwm-Velocity_Pwm;                            //===计算左轮电机最终PWM
 	Moto2=Balance_Pwm-Velocity_Pwm;                            //===计算右轮电机最终PWM
    MOTOR_Control(Moto1,Moto2);
	
} 
