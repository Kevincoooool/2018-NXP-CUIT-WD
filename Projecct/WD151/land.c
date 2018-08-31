#include "headfile.h"
#include "EM.h"
#include "Search.h"
#include "mymath.h"
int Pre_Out_Ring = 0,
    count_flage= 0,
    In_Ring = 0,
    Out_Ring=0,
    Enter_Ring = 0,
    Ring_flage = 0,
    Pre_Out_Ring1 = 0,
		Left_Go_Ring = 0,
    Right_Go_Ring = 0;
int16       FarSumToFlat = 0,FarDifToFlat = 0,FarDirToFlat = 0,CarDistanceToFlat = 0;

u8 ch_flage_R,ch_flage_L,Change_flage,Left_flage,Right_flage;


void Ring(void)
{
  if(mid_adc<=85&&mid_adc>=80&&ABS(left_adc + right_adc)>=175&&
		ABS(left_adc - right_adc)<=25&&ABS(LL_adc - RR_adc)>=3&&
	Pre_Out_Ring1==0&&Out_Ring==0)
  {
      if((RR_adc > LL_adc)&&Left_Go_Ring == 0)
      {
        Right_Go_Ring = 1;
      }
      else if((RR_adc < LL_adc)&&Right_Go_Ring == 0)
      {
        Left_Go_Ring = 1;
      }
       
  }
  if(((Right_Go_Ring==1&&left_adc >=95)||(Left_Go_Ring==1&&
		right_adc>=95))&& mid_adc>=95&&Ring_flage==0&&Out_Ring==0)
  {
    Ring_flage = 1;
    In_Ring = 1;
    if(Right_Go_Ring==1)
   {
//      led (LED0,LED_ON);
//      led (LED1,LED_OFF);
   }
   else if(Left_Go_Ring==1)
   {
//      led (LED1,LED_ON);
//      led (LED0,LED_OFF);
   }
  }
  if((Right_Go_Ring==1||Left_Go_Ring==1)&&mid_adc<=75)
  {
    count_flage++;
    if(count_flage>=300&&Ring_flage==0)
    {
      
      Left_Go_Ring = 0;
      Right_Go_Ring = 0;
      count_flage= 0;
      Right_flage=0;
      Left_flage=0;
    }
    if(count_flage>=300)
      count_flage=300;
  }
  if(In_Ring == 1&&ABS(left_adc - right_adc)<=28&&((Right_Go_Ring==1&&left_adc<=73)||(Left_Go_Ring==1&&right_adc<=73))&&mid_adc<=60&&Out_Ring==0)
  {
    Enter_Ring = 1;
    In_Ring = 0;
    Right_flage=0;
    Left_flage=0;
  }
  if(Enter_Ring==1&&(LL_adc + RR_adc)>=170&&ABS(LL_adc - RR_adc)<=58&&Out_Ring==0)//ABS(left_adc - right_adc)<=20&&((Right_Go_Ring==1&&left_adc>=85)||(Left_Go_Ring==1&&right_adc>=85))
  {
    Pre_Out_Ring = 1;
    Enter_Ring = 0;
    In_Ring = 0;
  }
  if(Pre_Out_Ring == 1&&Left_Go_Ring==1&&Out_Ring==0)
  {
        ch_flage_L = 1;
   }
   else if(Pre_Out_Ring == 1&&Right_Go_Ring==1&&Out_Ring==0)
    {
      
        ch_flage_R = 1;
    }
  else 
  {
    ch_flage_L =0;
    ch_flage_R =0;
  }
  if(Pre_Out_Ring == 1&&ABS(left_adc + right_adc)>=185&&ABS(left_adc - right_adc)<=20&&mid_adc>=88)//&&right_adc>=85
  {
    Out_Ring = 1;
    ch_flage_R = 0;
    ch_flage_L = 0;
    
  }
  if(Out_Ring==1&&ABS(left_adc - right_adc)<=20&&ABS(left_adc + right_adc)<=135&&RR_adc<=25&&LL_adc<=25)
  {
    Pre_Out_Ring = 0;
    count_flage= 0;
    In_Ring = 0;
    Out_Ring=0;
    Enter_Ring = 0;
    Ring_flage = 0;
    Pre_Out_Ring1 = 1;
    Pre_Out_Ring = 0;
    Left_Go_Ring = 0;
    Right_Go_Ring = 0;

  }
  if(Pre_Out_Ring1==1)
  {
    count_flage++;
    if(count_flage>=50)
    {
      Pre_Out_Ring1=0;
      count_flage=0;
    }
    if(count_flage>=50)
      count_flage=50;
  }
/*****************入环处理*********************/

   if(Right_Go_Ring == 1&&In_Ring == 1)//消除三角区对左电感或右电感的影响，切换电感
   {
      left_adc = (3*left_adc+7*mid_adc)/10;
      FarSumToFlat = left_adc  + right_adc;
      FarDifToFlat = right_adc  - left_adc;
      FarDirToFlat = (int)FarDifToFlat < 0 ? -1 : 1;
      Change_flage = 1;
   }
    else if(Left_Go_Ring == 1&&In_Ring == 1)
    {
      right_adc = (3*right_adc+7*mid_adc)/10;
      FarSumToFlat = left_adc  + right_adc;
      FarDifToFlat = right_adc  - left_adc;
      FarDirToFlat = (int)FarDifToFlat < 0 ? -1 : 1;  
      Change_flage = 1;
    }
    else if(Left_Go_Ring == 1&&ch_flage_L == 1)//
    {
      FarSumToFlat = left_adc  + mid_adc;
      FarDifToFlat = mid_adc  - left_adc;
      FarDirToFlat = (int)FarDifToFlat < 0 ? -1 : 1;  
      Change_flage = 2;
    }
    else if(Right_Go_Ring == 1&&ch_flage_R == 1)
    {
      FarSumToFlat = mid_adc  + right_adc;
      FarDifToFlat = right_adc  - mid_adc;
      FarDirToFlat = (int)FarDifToFlat < 0 ? -1 : 1;  
      Change_flage = 2;
    }
     else if(Left_Go_Ring == 1&&Out_Ring == 1)
    {
      left_adc = (4*left_adc+6*mid_adc)/10;
      FarSumToFlat = left_adc  + right_adc;
      FarDifToFlat = right_adc  - left_adc;
      FarDirToFlat = (int)FarDifToFlat < 0 ? -1 : 1;  
      Change_flage = 3;
    }
     else if(Right_Go_Ring == 1&&Out_Ring == 1)
    {
      right_adc = (4*right_adc+6*mid_adc)/10;
      FarSumToFlat = left_adc  + right_adc;
      FarDifToFlat = right_adc  - left_adc;
      FarDirToFlat = (int)FarDifToFlat < 0 ? -1 : 1; 
      Change_flage = 3;
    }
   else
  {
    FarSumToFlat = left_adc  + right_adc;  //保存水平传感器和值+RR_adc*1
    FarDifToFlat = right_adc  - left_adc;  //保存水平传感器差值
    
    Change_flage = 0;
  }
if(Change_flage == 1)
        CarDistanceToFlat =0 -  25*(int)(FarDifToFlat / FarSumToFlat* 100 );
else if(Change_flage == 2)
        CarDistanceToFlat =0 -  15*(int)(FarDifToFlat / FarSumToFlat* 100);
else if(Change_flage == 3)
        CarDistanceToFlat =0 -  5*(FarDifToFlat / FarSumToFlat* 100);
else
        CarDistanceToFlat =0 - FarDifToFlat / FarSumToFlat * 100;

        CarDistanceToFlat = CarDistanceToFlat * (200/ FarSumToFlat);
        if(CarDistanceToFlat > 1000) CarDistanceToFlat = 1000;
        if(CarDistanceToFlat < -1000) CarDistanceToFlat = -1000;
        Middle_Err = (int)CarDistanceToFlat;
}
