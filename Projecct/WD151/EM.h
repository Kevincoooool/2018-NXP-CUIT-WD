#ifndef _EM_H_
#define _EM_H_
#include "common.h"
extern u8 Ring_state;
extern u8 Ring_Flag;
extern int  left_adc,right_adc,mid_adc,LL_adc,RR_adc,MM_adc;
extern int16 left_delta, mid_delta, right_delta, LL_delta, RR_delta;
extern int Island_flag;
extern float dis_right_start,dis_left_start,angle_start;//±Í∂®æ‡¿Î
extern u8 Ring_state,ISLAND_dir;
extern int angle_buchang;
void EM_ADC(void);
void L_AD_Sample(void);
void Ring_Control(void);
void Ring_Stable(void);
extern uint16 Ring[3],Ring_Num;
extern uint16 Ring_Err[3];
extern uint16 Run_Distance[3];
extern float L_Slope,R_Slope,M_Slope,LL_Slope,RR_Slope;
extern int VoLa,VoLb;
extern uint16 jizhi_max[5],jizhi_min[5];
void Guiyi(void);
extern u8 Run_state;
void Run_Control(void);
extern uint16 Ring_En;
#endif
