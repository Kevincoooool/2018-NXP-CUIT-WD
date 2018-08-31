#ifndef _LED_h
#define _LED_h

#include "headfile.h"

#include "common.h"
#define BUTTON_UP        C2
#define BUTTON_LEFT      I6
#define BUTTON_RIGHT     D7
#define BUTTON_MID     	 D5 
#define BUTTON_DOWN      D6
//定义LED端口
#define LED0   I4
#define LED1   I1	
#define LED2   I0
#define LED3   G3	


/*********** 函数声明 ************/
void led_init(void);	//电机速度测量
void button_init(void);

#endif
