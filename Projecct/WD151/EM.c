#include "EM.h"
#include "Balance.h"
#include "control.h"
#include "loop.h"
#include "MPU6050.h"
#include "land.h"
#include "math.h"
#include "show.h"

/******************************************************************************************************************
2018年全国大学生恩智浦杯智能车竞赛
成都信息工程大学   成信WD队
作者QQ：97354734
文件作用:电感值采集和环岛状态机处理
*******************************************************************************************************************/
int left_adc_A,right_adc_A,mid_adc_A,LL_adc_A,RR_adc_A;
int left_adc,right_adc,mid_adc,LL_adc,RR_adc,MM_adc;
extern float Middle_Err;
#define FILTER_N 5
int16 filter_buf_L[FILTER_N];
int16 filter_buf_R[FILTER_N];
int16 filter_buf_M[FILTER_N];
int16 filter_buf_LL[FILTER_N];
int16 filter_buf_RR[FILTER_N];

void EM_ADC(void)
{
	//采集10次
	uint8 i = 0, j = 0;

	int16 filter_temp, filter_sum_L = 0, filter_sum_R = 0, filter_sum_M = 0;

	for(i = 0; i < FILTER_N; i++) 
	{
		filter_buf_L[i] = adc_once(ADC0_SE7,ADC_12bit);//左电感
		filter_buf_R[i] = adc_once(ADC0_SE15,ADC_12bit);//右电感
		filter_buf_M[i] = adc_once(ADC0_SE13,ADC_12bit);//中电感

	}
  
  // 采样值从小到大排列（冒泡法）
  for(j = 0; j < FILTER_N - 1; j++) {
    for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf_L[i] > filter_buf_L[i + 1]) {
        filter_temp = filter_buf_L[i];
        filter_buf_L[i] = filter_buf_L[i + 1];
        filter_buf_L[i + 1] = filter_temp;
      }
    }
  }
  
  for(j = 0; j < FILTER_N - 1; j++) {
    for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf_R[i] > filter_buf_R[i + 1]) {
        filter_temp = filter_buf_R[i];
        filter_buf_R[i] = filter_buf_R[i + 1];
        filter_buf_R[i + 1] = filter_temp;
      }
    }
  }
	for(j = 0; j < FILTER_N - 1; j++) {
		for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf_M[i] > filter_buf_M[i + 1]) {
        filter_temp = filter_buf_M[i];
        filter_buf_M[i] = filter_buf_M[i + 1];
        filter_buf_M[i + 1] = filter_temp;
      }
    }
  }

  // 去除最大最小极值后求平均
  for(i = 1; i < FILTER_N - 1; i++) 
  {
    filter_sum_L += filter_buf_L[i];
    filter_sum_R += filter_buf_R[i];
		filter_sum_M += filter_buf_M[i];

  }
  left_adc 	= filter_sum_L / (FILTER_N-2);
  right_adc = filter_sum_R / (FILTER_N-2);
	mid_adc 	= filter_sum_M / (FILTER_N-2);

	if(left_adc==0 && right_adc==0 && mid_adc==0) Run_Flag=0;//冲出赛道保护
	mid_adc = range_protect(mid_adc,0,350);
//	Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
	Middle_Err=4000.0f*(left_adc-right_adc)/((left_adc+right_adc+mid_adc)*mid_adc);
	Middle_Err = range_protect(Middle_Err,-100,100);
//	Ring_Control();
//	Middle_Err=1500.0f*(my_sqrt(left_adc)-my_sqrt(right_adc))/(left_adc+right_adc+mid_adc);

	
}

void L_AD_Sample(void)
{

	//angle_buchang = 1000*ABS(tan((imu_data.pit+2)*0.0174f));
	left_adc = adc_once(ADC0_SE6,ADC_12bit)/8;//左电感
	right_adc = adc_once(ADC0_SE12,ADC_12bit)/8;//右电感
	mid_adc = adc_once(ADC0_SE7,ADC_12bit)/8;//中电感

    mid_adc = range_protect(mid_adc,0,350);
//	Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
//	Middle_Err=3500.0f*(left_adc-right_adc)/((left_adc+right_adc+mid_adc)*mid_adc);
//	Middle_Err = range_protect(Middle_Err,-100,100);
//	Middle_Err=1400.0f*(my_sqrt(left_adc)-my_sqrt(right_adc))/(left_adc+right_adc+mid_adc);
//	Ring_Control();
	//Ring_Stable();
	Run_Control();
}

#define normal_Run    0
#define auto_Run     1
#define Wait_Ring      2 
#define Running      3 
#define Stop         4
u8 Run_state=0;
float Dis_Run = 0;
void Run_Control(void)//起跑状态机      
{
	switch(Run_state)
	{
		case auto_Run://直立对抗自动起步
			Run_Flag = 0;
			if(left_adc!=0 || right_adc!=0 || mid_adc!=0)
			{
				Run_state = Wait_Ring;
				Dis_Run = Distance;
			}
			break;
		case normal_Run://开机正常起步
			Middle_Err=4000.0f*(left_adc-right_adc)/((left_adc+right_adc+mid_adc)*mid_adc);
			Run_Flag = 1;
			Dis_Run = Distance;
			Run_state = Wait_Ring;
			if(left_adc==0 && right_adc==0 && mid_adc==0)  Run_state = Stop;//冲出赛道保护
		break;
		case Wait_Ring://等待环岛
			Middle_Err=4000.0f*(left_adc-right_adc)/((left_adc+right_adc+mid_adc)*mid_adc);
			Run_Flag = 1;
			if(Distance - Dis_Run >50)//起跑50cm开始检测环岛  防止信号源开机太大
				Run_state = Running;
			if(left_adc==0 && right_adc==0 && mid_adc==0)  Run_state = Stop;//冲出赛道保护
		break;
		case Running://正常跑
			Run_Flag = 1;
			Ring_Stable();
			if(left_adc==0 && right_adc==0 && mid_adc==0)  Run_state = Stop;//冲出赛道保护
		break;
		case Stop://停止
			Run_Flag = 0;
			
		break;
		default :break;
	}
}
#define No_Ring            0
#define Find_Right_Ring    1
#define Find_Left_Ring     2
#define Ready_Right_Ring   3
#define Ready_Left_Ring    4
#define Start_Right_Ring   5
#define Start_Left_Ring    6
#define In_Ring            7
#define Out_Ring           8
#define Real_Out_Ring      9

float angle_start;//标定距离
float Dis_Ring = 0;
u8 Ring_state=0;
void Ring_Control(void)//利用电感检测环岛  由于利用5个电感的阈值   不实用
{
	switch(Ring_state)
	{
		case No_Ring://第一步：判断是否出现环
				Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
		
				if(mid_adc>=230 && right_adc - left_adc>=15 &&right_adc >170 && left_adc >170 
					&& LL_adc - RR_adc>30 && RR_adc < 10)  //左环岛  先右电感大于左电感 然后左右电感值相等
				{
					Ring_state  = Find_Left_Ring;
					Dis_Ring = Distance;
				}
				else if(mid_adc>=230 &&left_adc - right_adc>=15 &&right_adc >170 && left_adc >170 
					&& RR_adc - LL_adc>30 &&  LL_adc < 10)//右环岛  先左电感大于右电感 然后左右电感值相等
				{
					Ring_state  = Find_Right_Ring;
					Dis_Ring = Distance;
				}
				else
					Ring_state  = No_Ring;
		break;
			
		case Find_Right_Ring://第二步：判断进环点
				Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
		
				if(ABS(left_adc - right_adc)<=10||(Distance - Dis_Ring >10&&Distance - Dis_Ring <30)) 
				{
					Dis_Ring = Distance;
					Ring_state  = Ready_Right_Ring;
				}
		break;
		case Find_Left_Ring://第二步：判断进环点
			Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
		
			if(ABS(left_adc - right_adc)<=10||(Distance - Dis_Ring >10&&Distance - Dis_Ring <30)) 
				{
					Dis_Ring = Distance;
					Ring_state  = Ready_Left_Ring;
				}
		break;
		case Ready_Right_Ring://第三步：准备进环
			Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
			if(right_adc > left_adc) //&&(Distance - Dis_Ring >30&&Distance - Dis_Ring <60)
				{
					Ring_state  = Start_Right_Ring;
					angle_start = imu_data.yaw;
				}
		break;	
		case Ready_Left_Ring://第三步：准备进环
			Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
			if(left_adc > right_adc )
				{
					Ring_state  = Start_Left_Ring;
					imu_data.inter_yaw = 0;
				}
		break;					
		case Start_Left_Ring://第四步：确认进环  开始记录当前角度

				Middle_Err = 70;
				imu_data.inter_yaw += GYRO_Real.Z*0.02;
				if(ABS(imu_data.inter_yaw)>=50.0f && (right_adc<160 && left_adc<160))//&& ((RR_adc>10 || LL_adc>10) && (right_adc<170 && left_adc<170))
				{
					Dis_Ring = Distance;
					Ring_state = In_Ring;
					imu_data.inter_yaw = 0;
				}
		break;	
		case Start_Right_Ring://第四步：确认进环  开始记录当前角度

				Middle_Err = -25;
				imu_data.inter_yaw += GYRO_Real.Z*0.02;
				if(ABS(imu_data.inter_yaw)>=50.0f && (right_adc<160 && left_adc<160))//&& ((RR_adc>10 || LL_adc>10) && (right_adc<170 && left_adc<170))
				{
					Dis_Ring = Distance;
					Ring_state = In_Ring;
					imu_data.inter_yaw = 0;
				}
		break;
		case In_Ring://第五步：环岛中  记录陀螺仪积分角度变化大于360度
				Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
				imu_data.inter_yaw += GYRO_Real.Z*0.02;
				if(ABS(imu_data.inter_yaw)>=360.0f )//&&Distance - Dis_Ring >200&&Distance - Dis_Ring <250
				{
					Ring_state = Out_Ring;
					imu_data.inter_yaw= 0;
				}
		break;	
		case Out_Ring://第六步：出环  回到无环状态
				Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
				if(mid_adc<=240 &&mid_adc>=150 && ABS(left_adc - right_adc)<=20&&left_adc<=150 &&right_adc<=150 )  //出环条件
					Ring_state  = No_Ring;
				
				imu_data.inter_yaw= 0;
			
		break;
		default:break;
			
	}
	
}
u8 Ring_Flag = 0;

uint16 Ring[3],Ring_Num;
uint16 Ring_Err[3];
uint16 Ring_Dis[3];
uint16 Run_Distance[3];
uint16 Ring_En = 1;
void Ring_Stable(void)//只检测三个电感的阈值    任意赛道调一下就行
{
	switch(Ring_state)
	{
		case No_Ring://第一步：判断是否出现环
			//Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
			Middle_Err=4000.0f*(left_adc-right_adc)/((left_adc+right_adc+mid_adc)*mid_adc);
			if(mid_adc>=160 &&right_adc >180 && left_adc >180)  
			{
	
				if (Ring_Flag ==0 && Ring[0] ==1)
				{
					Ring_state  = Find_Right_Ring;	
				}
				else if (Ring_Flag ==0 && Ring[0] ==0)
				{
					Ring_state  = Find_Left_Ring;	
				}
				else if (Ring_Flag ==1 && Ring[1] ==1)
				{
					Ring_state  = Find_Right_Ring;
				}
				else if (Ring_Flag ==1 && Ring[1] ==0)
				{
					Ring_state  = Find_Left_Ring;
				}
				else if (Ring_Flag ==2 && Ring[2] ==1)
				{
					Ring_state  = Find_Right_Ring;
						
				}
				else if (Ring_Flag ==2 && Ring[2] ==0)
				{
					Ring_state  = Find_Left_Ring;
				}
			}

			else
				Ring_state  = No_Ring;
		break;
				
		case Find_Right_Ring://第二步：判断进环点
				//Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
		Middle_Err=3500.0f*(left_adc-right_adc)/((left_adc+right_adc+mid_adc)*mid_adc);
				if(ABS(left_adc - right_adc)>=30) 
				{
					Dis_Ring = Distance;
					Ring_state  = Ready_Right_Ring;
				}
		break;
		case Find_Left_Ring://第二步：判断进环点
			//Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
	Middle_Err=3500.0f*(left_adc-right_adc)/((left_adc+right_adc+mid_adc)*mid_adc);
			if(ABS(left_adc - right_adc)>=30) 
				{
					Dis_Ring = Distance;
					Ring_state  = Ready_Left_Ring;
				}
		break;
		case Ready_Right_Ring://第三步：准备进环点  检测阈值开始变大的点开始的距离
			//Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
		Middle_Err=3500.0f*(left_adc-right_adc)/((left_adc+right_adc+mid_adc)*mid_adc);
			Run_Distance[Ring_Flag] = 95 - Ring_Err[Ring_Flag];
			
			if(Distance - Dis_Ring >Run_Distance[Ring_Flag]) //&&(Distance - Dis_Ring >30&&Distance - Dis_Ring <60)
				{
					Ring_state  = Start_Right_Ring;
					Dis_Ring = Distance;
					imu_data.inter_yaw = 0;
				}
		break;	
		case Ready_Left_Ring://第三步：准备进环点  检测阈值开始变大的点开始的距离
			//Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
		Middle_Err=3500.0f*(left_adc-right_adc)/((left_adc+right_adc+mid_adc)*mid_adc);
			Run_Distance[Ring_Flag] = 95 - Ring_Err[Ring_Flag];
		
			if(Distance - Dis_Ring >Run_Distance[Ring_Flag])
				{
					Ring_state  = Start_Left_Ring;
					Dis_Ring = Distance;
					imu_data.inter_yaw = 0;
				}
		break;		
		case Start_Left_Ring://第四步：开始进环  开始记录当前角度  以固定偏差走一段距离和角度

				Middle_Err = Ring_Err[Ring_Flag];
				
				imu_data.inter_yaw += GYRO_Real.Z*0.02f;
				
				if(ABS(imu_data.inter_yaw)>=60.0f &&(Distance - Dis_Ring >50))//&& (right_adc<140 && left_adc<140 && mid_adc<170)
				{
					Dis_Ring = Distance;
					Ring_state = In_Ring;
					imu_data.inter_yaw = 0;
				}
		break;	
		case Start_Right_Ring://第四步：开始进环  开始记录当前角度  以固定偏差走一段距离和角度
			
				Middle_Err = -Ring_Err[Ring_Flag];
		
				imu_data.inter_yaw += GYRO_Real.Z*0.02f;
				if(ABS(imu_data.inter_yaw)>=60.0f &&(Distance - Dis_Ring >50))//&& ((RR_adc>10 || LL_adc>10) && (right_adc<170 && left_adc<170))
				{
					Dis_Ring = Distance;
					Ring_state = In_Ring;
					imu_data.inter_yaw = 0;
				}
		break;
		case In_Ring://第四步：环岛中  记录陀螺仪积分角度变化大于360度
				//Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
		Middle_Err=3700.0f*(left_adc-right_adc)/((left_adc+right_adc+mid_adc)*mid_adc);
				imu_data.inter_yaw += GYRO_Real.Z*0.02f;
				if(ABS(imu_data.inter_yaw)>=300.0f&&Distance - Dis_Ring >200 )//&&Distance - Dis_Ring >200&&Distance - Dis_Ring <250
				{
					Ring_state = Out_Ring;
					imu_data.inter_yaw= 0;
				}
		break;	
				
		case Out_Ring://第五步：出环  两边的电感一定会出现极大值
			//Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
		Middle_Err=3700.0f*(left_adc-right_adc)/((left_adc+right_adc+mid_adc)*(mid_adc/2));
			if(right_adc>=380 ||left_adc>=380  ) //出环条件
				{
					Ring_state  = Real_Out_Ring;
				}
		break;
				
		case Real_Out_Ring://第六步：出环  三个电感值都减小后  回到无环状态
			//Middle_Err=80.0f*(left_adc-right_adc)/(left_adc+right_adc+mid_adc);
		Middle_Err=3700.0f*(left_adc-right_adc)/((left_adc+right_adc+mid_adc)*(mid_adc/1.5));
			if(mid_adc<=180 && right_adc<170 && left_adc<=170 ) //出环条件
			{
				Ring_state  = No_Ring;
				
				if (Ring_Num == 1)
					Ring_Flag=0;
				if (Ring_Num == 2)
				{
					Ring_Flag++;
					if (Ring_Flag>=2)
						Ring_Flag = 0;
				}
				if (Ring_Num == 3)
				{
					Ring_Flag++;
					if (Ring_Flag>=3)
						Ring_Flag = 0;
				}
			}
		break;
		default:break;		
	}
	Middle_Err = range_protect(Middle_Err,-100,100);
}

