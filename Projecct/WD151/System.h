#ifndef __SYSTEM_H__
#define __SYSTEM_H__
#include "common.h"
//#include "include.h"

extern uint8 Send_OK;
extern uint8 System_OK;

void System_Init(void);		 //所有模块初始化
void My_DEBUG(void);
extern char Mode_Set;
#endif
