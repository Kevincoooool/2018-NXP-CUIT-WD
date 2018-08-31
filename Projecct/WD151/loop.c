#include "loop.h"
#include "MPU6050.h"
#include "ADRC.h"
#include "Balance.h"
#include "led.h"
#include "Control.h"
#include "filter.h"
#include "ANO_Data_Transfer.h"
#include "show.h"
#include "PID.h"
#include "My_I2C.h"
#include "EM.h"
#include "Search.h"
#include "ADRC.h"
/******************************************************************************************************************
2018年全国大学生恩智浦杯智能车竞赛
成都信息工程大学   成信WD队
作者QQ：97354734
文件作用:任务调度
*******************************************************************************************************************/
volatile uint32_t sysTickUptime = 0;
float time_up[12],time_sum1 = 0;

static void Duty_2ms(void)
{
	L_AD_Sample();
	MPU6050_GetData(&GYRO, &ACC);	// 读取陀螺仪数据
	Data_steepest();              //原始数据梯度下降滤波
}

static void Duty_4ms(void)
{
	
	Theory_Duty += -PID_Increase(&Ang_gyro_PID, Ang_gyro, (int32)(GYRO_Real.Y*10),(int32)Tar_Ang_Vel.Y);	// 直立内环角速度环
	Theory_Duty = range_protect(Theory_Duty, -950, 950);                                        // 直立PWM限幅

	Direct_Parameter = PID_Realize(&Turn_gyro_PID, Turn_gyro, (int32)(GYRO_Real.Z*100),(int32)Radius*Speed_Min);	// 转向环左正右负(int32)Radius*Speed_Min
	Direct_Parameter = range_protect(Direct_Parameter, -1100, 1100);

	Direct_Last = Direct_Last*0.3 + Direct_Parameter*0.7;	// 更新上次角速度环结果
	MOTOR_Duty_Left  = (Theory_Duty - Direct_Last);	// 左右电机根据转向系数调整差速
	MOTOR_Duty_Right = (Theory_Duty + Direct_Last);	
	
	if (Run_Flag&&sysTickUptime>5000)//开机等待互补滤波初始化5秒后启动电机
		MOTOR_Control(MOTOR_Duty_Left, MOTOR_Duty_Right);	// 控制左右电机

	else
	{
		if (Stop_Flag)
		{
			if (Speed_Now > 60) MOTOR_Control(-850, -850);
			else                MOTOR_Control(0, 0);
		}
		else
			MOTOR_Control(0,0);
	}	
	
}

static void Duty_8ms(void)
{
	
	IMU_update(0.008f,&(sensor.Gyro_deg), &(sensor.Acc_mmss),&imu_data); //姿态解算
	Speed_Measure();                                                     //速度测量
	Distance += Speed_Now/55.0f;                     
	Speed_Calculate();                                                  //根据当前速度判断转向PID参数
	Radius = -PlacePID_Control(&Turn_PID, Turn[Fres], Middle_Err, 0);  	//转向外环PID
	Road_Find();
	Tar_Ang_Vel.Y = -PID_Realize(&Angle_PID, Angle,imu_data.pit*100, ADRC_SPEED_Controller.x1);	/* 角度环加到角速度环上串级控制 *///ADRC_SPEED_Controller.x1
	Tar_Ang_Vel.Y = range_protect(Tar_Ang_Vel.Y,-1500, 1500);	// 注意正负号
	//ANO_DT_Data_Exchange();//传输数据到匿名上位机   加上后将对控制周期产生影响  调试时可加上方便查看波形
}

static void Duty_40ms(void)    
{

	Target_Angle.Y = -PID_Realize(&Speed_PID, Speed, ADRC_SPEED_MIN_Controller.x1,Speed_Set);/* 速度环加到角度环上串级控制 ADRC_SPEED_MIN_Controller.x1 Speed_Now*/
	Target_Angle.Y +=1400;        //重心为14度
	if (ABS(Middle_Err>30)||sysTickUptime<5500)//弯道或者起跑前几秒满速
		Target_Angle.Y = -500;
	else
		Target_Angle.Y = range_protect(Target_Angle.Y,Speed_Limit_Min, Speed_Limit_Max);	//  Speed_Limit_Min, Speed_Limit_Max

	Fhan_ADRC(&ADRC_SPEED_Controller,Target_Angle.Y); //对速度环的输出量进行TD

}
static void Duty_100ms(void) 
{
	gpio_turn(I0);
	if (OLED_Refresh)
	OLED_Draw_UI();
	Check_BottonPress();
	BAT_CHECK();
}

//系统任务配置，创建不同执行周期的“线程”
static sched_task_t sched_tasks[] = 
{
	{Duty_100ms ,  100, 0},
	{Duty_40ms  ,   40, 0},
	{Duty_8ms   ,    8, 0},
	{Duty_4ms   ,    4, 0},
	{Duty_2ms   ,    2, 0}

};
//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))

//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Loop_Run(void)
{
	uint8_t index = 0;
	
	//循环判断所有线程，是否应该执行
	for(index=0;index < TASK_NUM;index++)
	{
		//获取系统当前时间，单位MS --
		uint32_t tnow = sysTickUptime;
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		if(tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{
			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = tnow;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func();
		}
	}
}


	

