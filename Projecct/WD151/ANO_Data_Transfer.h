#ifndef __ANO_DataTransfer_H
#define __ANO_DataTransfer_H

#include "common.h"

typedef struct 
{
	u8 msg_id;
	u8 msg_data;
	u8 send_version;
	u8 send_status;
	u8 send_senser;
	u8 send_senser2;
	u8 send_pid;
	u8 send_pid1;
	u8 send_pid2;
	u8 send_pid3;
	u8 send_rcdata;
	u8 send_offset;
	u8 send_motopwm;
	u8 send_power;
	u8 send_user;
	u8 send_speed;
	u8 send_location;

}dt_flag_t;

extern dt_flag_t f;
void ANO_DT_Send_Check(u8 head, u8 check_sum);
void ANO_DT_Data_Exchange(void);
void ANO_DT_Data_Receive_Prepare(u8 data);
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void ANO_DT_Send_Senser2(s32 bar_alt,u16 csb_alt);
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_Power(u16 votage, u16 current, u8 flag0, u8 flag1, u16 flag2, u16 flag3);
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void ANO_DT_Send_PID(u8 group,s16 p1_p,s16 p1_i,s16 p1_d,s16 p2_p,s16 p2_i,s16 p2_d,s16 p3_p,s16 p3_i,s16 p3_d);
void ANO_DT_Send_User(void);
void ANO_DT_Send_Speed(float,float,float);
void ANO_DT_Send_Location(void);

#endif

