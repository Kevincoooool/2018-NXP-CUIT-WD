#ifndef _SEARCH_H_
#define _SEARCH_H_
#include "common.h"
extern uint8 speed_low;
extern uint8 speed_mid;
extern uint8 speed_high;
extern uint16 speed_higher;
extern float  Middle_Err;
extern uint8 Str_Flag;
extern uint8 Curve_Flag;
extern u8 Road_state;
void Road_Find(void);
void str_judg(void);
void island_judg(void);
#endif
