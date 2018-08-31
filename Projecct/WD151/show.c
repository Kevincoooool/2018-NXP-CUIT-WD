#include "show.h"
#include "led.h"
#include "MyBalance.h"
#include "My_I2C.h"
#include "Balance.h"
#include "Search.h"
#include "PID.h"
#include "control.h"
#include "MPU6050.h"
#include "EM.h"
/******************************************************************************************************************
2018年全国大学生恩智浦杯智能车竞赛
成都信息工程大学   成信WD队
作者QQ：97354734
文件作用:OLED及按键
*******************************************************************************************************************/
int mark=0;
u8 func=0;
int   Start_Cnt=0;
uint8 Page_Index=1,OLED_Refresh=0;
//对应不同的页面


void OLED_Draw_UI(void)  //画出界面
{ 
	if(mark<=7) Page_Index=1;
	if(mark>7&&mark<=14) Page_Index=2;
	if(mark>14) Page_Index=3;

	if(Page_Index==1)
	{
    OLED_P6x8Str(0,0,"CarAngle");
    OLED_Print_Num1(80,0,imu_data.pit);
	OLED_P6x8Str(0,1,"BAT:");
    OLED_Print_Num1(80, 1,adc_once(ADC0_SE4,ADC_12bit)*3.3*12*10/2/4096);
	OLED_P6x8Str(0,2,"12V:");
    OLED_Print_Num1(80, 2,adc_once(ADC0_SE5,ADC_12bit)*3.3*12*10/2/4096);
	
//	OLED_P6x8Str(0,1,"RIGHT:");
//    OLED_Print_Num1(80, 1,MOTOR_Speed_Right);
//	OLED_P6x8Str(0,2,"LEFT:");
//    OLED_Print_Num1(80, 2,MOTOR_Speed_Left);
//	
    OLED_P6x8Str(0,3,"AD_L:");
    OLED_Print_Num1(80,3,left_adc );
    OLED_P6x8Str(0,4,"AD_M:");
    OLED_Print_Num1(80,4, mid_adc);
    OLED_P6x8Str(0,5,"AD_R:");
    OLED_Print_Num1(80,5,right_adc );
	OLED_P6x8Str(0,6,"Err:");
    OLED_Print_Num1(80,6,Middle_Err );
	OLED_P6x8Str(0,7,"Stop:");
    OLED_Print_Num1(80,7,Run_Stop );

    OLED_P6x8Str(115,mark,"<-");

	}
	else if (Page_Index==2)
	{
		OLED_P6x8Str(0,0,"Ring_Num:");
		OLED_Print_Num1(80,0,Ring_Num);
		OLED_P6x8Str(0,1,"Ring_1:");
		OLED_Print_Num1(80,1,Ring[0]);
		OLED_P6x8Str(0,2,"Ring_2:");
		OLED_Print_Num1(80, 2,Ring[1]);
		OLED_P6x8Str(0,3,"Ring_3:");
		OLED_Print_Num1(80, 3,Ring[2]);
		
		OLED_P6x8Str(0,4,"Ring_Err_1:");
		OLED_Print_Num1(80,4,Ring_Err[0] );
		OLED_P6x8Str(0,5,"Ring_Err_2:");
		OLED_Print_Num1(80,5, Ring_Err[1]);
		OLED_P6x8Str(0,6,"Ring_Err_3:");
		OLED_Print_Num1(80,6,Ring_Err[2] );

		OLED_P6x8Str(115,mark-8,"<-");
	}
	else if (Page_Index==3)
	{
		OLED_P6x8Str(0,0,"Speed_Set:");
		OLED_Print_Num1(80,0,Speed_Set);
		OLED_P6x8Str(0,1,"speed_low:");
		OLED_Print_Num1(80,1,speed_low);
		OLED_P6x8Str(0,2,"speed_mid:");
		OLED_Print_Num1(80,2,speed_mid);
		OLED_P6x8Str(0,3,"speed_high:");
		OLED_Print_Num1(80,3,speed_high);
		OLED_P6x8Str(0,4,"speed_higher:");
		OLED_Print_Num1(80,4,speed_higher);

		OLED_P6x8Str(115,mark-15,"<-");
	}

}

/*
 * 检测按键是否按下
 */
void Check_BottonPress()
{
    if(gpio_get(BUTTON_MID)==0)
   {
       systick_delay_ms(2);
      if(gpio_get(BUTTON_MID)==0)
      {    
				OLED_Refresh=!OLED_Refresh;
				OLED_Fill(0);//清屏
				if(Run_Flag==0&&!OLED_Refresh)
				{
					Balance_Init();
					Run_Flag=!OLED_Refresh;
				}
      }
      while(gpio_get(BUTTON_MID)==0);  //直到按键松开再运行
   } 
   
   //按键2 Left_L
     if(gpio_get(BUTTON_LEFT)==0&&OLED_Refresh)
   {
      //去抖
      systick_delay_ms(2);
      if(gpio_get(BUTTON_LEFT)==0)
      {
			if (mark==7) Run_Stop=0;
			if (mark==9) speed_low--;
			if (mark==10) speed_mid--;
			if (mark==11) speed_high--;
			if (mark==12) speed_higher--;


      }
      //while(gpio_get(BUTTON_LEFT)==0);//直到按键松开再运行
   } 
   //按键6 Right_L
     if(gpio_get(BUTTON_RIGHT)==0&&OLED_Refresh)
   {
      //去抖
      systick_delay_ms(2);
      if(gpio_get(BUTTON_RIGHT)==0)
      {
        //Speed_Set++;
				if (mark==7) Run_Stop=1;
				if (mark==9) speed_low++;
				if (mark==10) speed_mid++;
				if (mark==11) speed_high++;
				if (mark==12) speed_higher++;

        // OLED_Fill(0);//清屏 
        
      }
      //while(gpio_get(BUTTON_RIGHT)==0);      //直到按键松开再运行
   }
   //按键3 up
     if(gpio_get(BUTTON_UP)==0&&OLED_Refresh)
   {

		systick_delay_ms(2);
      if(gpio_get(BUTTON_UP)==0)
      {
        mark--;
        if(mark<1)
        {  
				mark=0;
				}
       OLED_Fill(0);//清屏
      }   
      //while(gpio_get(BUTTON_UP)==0);//直到按键松开再运行  
   }
   //按键4 down
     if(gpio_get(BUTTON_DOWN)==0&&OLED_Refresh)
   {
     systick_delay_ms(2);
      if(gpio_get(BUTTON_DOWN)==0)
      {
				mark++;
				if(mark>19) 
				{  
					mark=19;
				}
				
				OLED_Fill(0);//清屏
      }
      //while(gpio_get(BUTTON_DOWN)==0);  //直到按键松开再运行
   }
}

void Ring_Set(void)
{

   OLED_P6x8Str(115,mark,"<-");
   //按键2 Left_L
   if(gpio_get(BUTTON_LEFT)==0)
   {
      //去抖
		systick_delay_ms(2);
		if(gpio_get(BUTTON_LEFT)==0)
		{
			if (mark==0) 
			{
				Ring_Num--;
				if (Ring_Num<=0)Ring_Num=0;
			}
			if (mark==1) Ring[0]=0;
			if (mark==2) 
			{
				Ring_Err[0]--;
				if (Ring_Err[0]<=0)Ring_Err[0]=0;
				
			}
			if (mark==3) Ring[1]=0;
			
			if (mark==4)
			{
				Ring_Err[1]--;
				if (Ring_Err[1]<=0)Ring_Err[1]=0;
			}
			if (mark==5) Ring[2]=0;
			if (mark==6)
			{
				Ring_Err[2]--;
				if (Ring_Err[2]<=0)Ring_Err[2]=0;
			}

      }
     if (mark==0) while(gpio_get(BUTTON_LEFT)==0);//直到按键松开再运行
   } 
   //按键6 Right_L
	if(gpio_get(BUTTON_RIGHT)==0)
	{
      //去抖
		systick_delay_ms(2);
		if(gpio_get(BUTTON_RIGHT)==0)
		{
			if (mark==0) Ring_Num++;
			if (mark==1) Ring[0]=1;
			if (mark==2) Ring_Err[0]++;
			if (mark==3) Ring[1]=1;
			if (mark==4) Ring_Err[1]++;
			if (mark==5) Ring[2]=1;
			if (mark==6) Ring_Err[2]++;

        
      }
     if (mark==0) while(gpio_get(BUTTON_RIGHT)==0);      //直到按键松开再运行
   }
   //按键3 up
     if(gpio_get(BUTTON_UP)==0)
   {

		systick_delay_ms(2);
      if(gpio_get(BUTTON_UP)==0)
      {
        mark--;
        if(mark<1)
        {  
				mark=0;
				}
       OLED_Fill(0);//清屏
      }   
      while(gpio_get(BUTTON_UP)==0);//直到按键松开再运行  
   }
   //按键4 down
     if(gpio_get(BUTTON_DOWN)==0)
   {
     systick_delay_ms(2);
      if(gpio_get(BUTTON_DOWN)==0)
      {
				mark++;
				if(mark>7) 
				{  
					mark=7;
				}
				
				OLED_Fill(0);//清屏
      }
      while(gpio_get(BUTTON_DOWN)==0);  //直到按键松开再运行
   }
}
