/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		中断文件
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/



#include "isr.h"
#include "loop.h"
#include "Balance.h"
#include "ANO_Data_Transfer.h"
#include "control.h"
#include "PID.h"
//#include "MPU6050.h"
//#include "My_I2C.h"
u32 time[7],time_sum;
void PIT_CH0_IRQHandler(void)
{

	PIT_FLAG_CLR(pit0);//清除定时器中断标志位
}

void PIT_CH1_IRQHandler(void)
{
    
	static uint16 Time = 0;
	sysTickUptime++;
	if(sysTickUptime>5000)
	{
		gpio_set(I1,0);
	}
	Start_Control();
//	if(sysTickUptime<8000)//开跑前3s迅速到达期望速度 之后减弱速度控制
//	{
//		Speed[0] = 50;
//	}
//	else Speed[0] = 10;
	if (Run_OK==0)
	{
		Time++;
		if (Time >= 8000)			//延时4s
		{
			Time=0;
			Run_OK=1;
		}
	}
	else
	{
		if (Stop_Flag==1)			//延时
		{
			Time++;
			if (Time >= 1000)
			{
				Time = 0;
				Stop_Flag=0;
			}

		}
	}
	PIT_FLAG_CLR(pit1);
}

void IRQ_IRQHandler(void)
{
    CLEAR_IRQ_FLAG;
}


void KBI0_IRQHandler(void)
{
    CLEAN_KBI0_FLAG;
    
}

void UART1_IRQHandler(void)
{
    char ch;

    if(UART1->S1 & UART_S1_RDRF_MASK)   //接收数据寄存器满
    {
			ch  =   UART1->D; 
			ANO_DT_Data_Receive_Prepare(ch);
			
    }
}
/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了周期定时器 通道0得中断
void PIT_CH0_IRQHandler(void)
{
    ;
}
记得进入中断后清除标志位

FTMRE_IRQHandler      
PMC_IRQHandler        
IRQ_IRQHandler        
I2C0_IRQHandler       
I2C1_IRQHandler       
SPI0_IRQHandler       
SPI1_IRQHandler       
UART0_IRQHandler 
UART1_IRQHandler 
UART2_IRQHandler 
ADC0_IRQHandler       
ACMP0_IRQHandler      
FTM0_IRQHandler       
FTM1_IRQHandler       
FTM2_IRQHandler       
RTC_IRQHandler        
ACMP1_IRQHandler      
PIT_CH0_IRQHandler    
PIT_CH1_IRQHandler    
KBI0_IRQHandler       
KBI1_IRQHandler       
Reserved26_IRQHandler 
ICS_IRQHandler        
WDG_IRQHandler        
PWT_IRQHandler        
MSCAN_Rx_IRQHandler   
MSCAN_Tx_IRQHandler   
*/



