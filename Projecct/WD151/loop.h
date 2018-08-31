#ifndef _LOOP_H_
#define _LOOP_H_

#include "common.h"
extern volatile uint32_t sysTickUptime;
typedef struct
{
	void(*task_func)(void);
	uint16_t interval_ticks;//执行时间间隔
	uint32_t last_run;//最后一次运行时间
}sched_task_t;

extern void Loop_Run(void);
void Scheduler_Setup(void);
#endif

