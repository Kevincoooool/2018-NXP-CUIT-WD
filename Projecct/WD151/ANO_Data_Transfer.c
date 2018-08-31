

//#include "include.h"
#include "ANO_Data_Transfer.h"
#include "filter.h"
#include "Balance.h"
#include "PID.h"
#include "EM.h"
#include "ADRC.h"
#include "Control.h"
#include "MPU6050.h"
#include "My_I2C.h"
#include "Search.h"
/******************************************************************************************************************
2018年全国大学生恩智浦杯智能车竞赛
成都信息工程大学   成信WD队
作者QQ：97354734
文件作用:数据回传上位机
*******************************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (u8 *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (u8 *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (u8 *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (u8 *)(&dwTemp) + 3) )
u16 save_pid_en = 0;
dt_flag_t f;					//需要发送数据的标志
u8 data_to_send[50];			//发送数据缓存

//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数

void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	uart_putbuff(uart1,dataToSend,length);
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每6ms发送一次传感器数据至上位机，即在此函数内实现(cnt = 6/2 = 3)
//此函数应由用户每2ms调用一次
u8 cnt = 0,yaw_lock = 0,Send_Check = 0;

extern _fix_inte_filter_st wcz_spe_fus;
void ANO_DT_Data_Exchange(void)
{

	if(f.send_pid)
	{
		cnt++;
		switch(cnt)
		{
			case 1:
					ANO_DT_Send_PID(4,Speed[0]*100,Speed_Set,Speed[2]*100,Angle[0]*100,Angle[1]*100,Angle[2]*100,Ang_gyro[0]*100,Ang_gyro[1]*1000,Ang_gyro[2]*100);	
			break;
			case 2:
					ANO_DT_Send_PID(5,Turn_gyro[0]*1000,Turn_gyro[1]*1000,Turn_gyro[2]*1000,Turn[0][0],Turn[0][1],Turn[0][2],Turn[1][0],Turn[1][1],Turn[1][2]);
			break;
			case 3:	
					ANO_DT_Send_PID(6,Turn[2][0],Turn[2][1],Turn[2][2],Turn[3][0],Turn[3][1],Turn[3][2],Turn[4][0],Turn[4][1],Turn[4][2]);
			cnt = 0;	
			f.send_pid = 0;	
			break;
			default:break;
		}
	}

	else
	{
		cnt++;
		switch(cnt)
		{

			case 1:	ANO_DT_Send_Status(imu_data.rol,imu_data.pit,imu_data.yaw,Speed_Min*100,2,0);	
				break;
			case 2:	//ANO_DT_Send_Senser(Direct_Parameter,Direct_Last,imu_data.w_acc.z,GYRO_Real.X,GYRO_Real.Y,GYRO_Real.Z,left_adc,mid_adc,right_adc);
				ANO_DT_Send_Senser(Middle_Err,Road_state,Ring_state,imu_data.w_acc.x,Speed_Now,Distance,left_adc,mid_adc,right_adc);
			//ANO_DT_Send_Senser(Tar_Ang_Vel.Y,ADRC_GYRO_Controller.x1,Target_Angle.Y ,ADRC_SPEED_Controller.x1,GYRO_Real.Y,ADRC_SPEED_MIN_Controller.x1,Speed_Now,ADRC_GYRO_Controller.e2,ADRC_GYRO_Controller.u);
				cnt = 0;	break;
			
		}
	}
	
}

void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	u8 i;
	u8 sum = 0;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	for(i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用

extern u16 save_pid_en;

void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
		switch( *(data_buf+2) )
	{
			
		case 0X02:
			if(*(data_buf+4)==0X01)
			{
				f.send_pid = 1;
				cnt = 0;
			}
			if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
			Speed[0] = 20;
			Speed[1] = 0;
			Speed[2] = 3;
			Angle[0] = 0.3;
			Angle[1] = 0;
			Angle[2] = 0.1;
			Ang_gyro[0] 	= 0.3;
			Ang_gyro[1] 	= 0.023;
			Ang_gyro[2]	= 0;
		}

		break;
			
		case 0X10:
			Speed[0] = ( (int16)(*(data_buf+4)<<8)|*(data_buf+5) ) /100.0;
			Speed_Set= ( (int16)(*(data_buf+6)<<8)|*(data_buf+7) ) ;
			Speed[2] = ( (int16)(*(data_buf+8)<<8)|*(data_buf+9) ) /100.0;
			Angle[0] = ( (int16)(*(data_buf+10)<<8)|*(data_buf+11) ) /100.0;
			Angle[1] = ( (int16)(*(data_buf+12)<<8)|*(data_buf+13) ) /100.0;
			Angle[2] = ( (int16)(*(data_buf+14)<<8)|*(data_buf+15) ) /100.0;
			Ang_gyro[0] = ( (int16)(*(data_buf+16)<<8)|*(data_buf+17) ) /100.0;
			Ang_gyro[1] = ( (int16)(*(data_buf+18)<<8)|*(data_buf+19) ) /1000.0;
			Ang_gyro[2]	= ( (int16)(*(data_buf+20)<<8)|*(data_buf+21) ) /100.0;
			ANO_DT_Send_Check(*(data_buf+2),sum);
			save_pid_en = 1;
		break;
		case 0X11:
			Turn_gyro[0] = ( (int16)(*(data_buf+4)<<8)|*(data_buf+5) ) /1000.0;
			Turn_gyro[1] = ( (int16)(*(data_buf+6)<<8)|*(data_buf+7) ) /1000.0;
			Turn_gyro[2] = ( (int16)(*(data_buf+8)<<8)|*(data_buf+9) ) /1000.0;
			Turn[0][0] = ( (int16)(*(data_buf+10)<<8)|*(data_buf+11) ) ;
			Turn[0][1] = ( (int16)(*(data_buf+12)<<8)|*(data_buf+13) );
			Turn[0][2] = ( (int16)(*(data_buf+14)<<8)|*(data_buf+15) );
			Turn[1][0] = ( (int16)(*(data_buf+16)<<8)|*(data_buf+17) );
			Turn[1][1] = ( (int16)(*(data_buf+18)<<8)|*(data_buf+19) ) ;
			Turn[1][2]	= ( (int16)(*(data_buf+20)<<8)|*(data_buf+21) );
			ANO_DT_Send_Check(*(data_buf+2),sum);
			save_pid_en = 1;
		break;
		case 0X12:
			Turn[2][0] = ( (int16)(*(data_buf+4)<<8)|*(data_buf+5) );
			Turn[2][1] = ( (int16)(*(data_buf+6)<<8)|*(data_buf+7) ) ;
			Turn[2][2] = ( (int16)(*(data_buf+8)<<8)|*(data_buf+9) );
			Turn[3][0] = ( (int16)(*(data_buf+10)<<8)|*(data_buf+11) ) ;
			Turn[3][1] = ( (int16)(*(data_buf+12)<<8)|*(data_buf+13) );
			Turn[3][2] = ( (int16)(*(data_buf+14)<<8)|*(data_buf+15) );
			Turn[4][0] = ( (int16)(*(data_buf+16)<<8)|*(data_buf+17) );
			Turn[4][1] = ( (int16)(*(data_buf+18)<<8)|*(data_buf+19) ) ;
			Turn[4][2]	= ( (int16)(*(data_buf+20)<<8)|*(data_buf+21) );
			ANO_DT_Send_Check(*(data_buf+2),sum);
			save_pid_en = 1;
		break;
		case 0X13:

			ANO_DT_Send_Check(*(data_buf+2),sum);
			save_pid_en = 1;
		break;
		case 0X14:
//			Turn[2][0] = ( (int16)(*(data_buf+4)<<8)|*(data_buf+5) );
//			Turn[2][1] = ( (int16)(*(data_buf+6)<<8)|*(data_buf+7) ) ;
//			Turn[2][2] = ( (int16)(*(data_buf+8)<<8)|*(data_buf+9) );
//			Turn[3][0] = ( (int16)(*(data_buf+10)<<8)|*(data_buf+11) ) ;
//			Turn[3][1] = ( (int16)(*(data_buf+12)<<8)|*(data_buf+13) );
//			Turn[3][2] = ( (int16)(*(data_buf+14)<<8)|*(data_buf+15) );
//			Turn[4][0] = ( (int16)(*(data_buf+16)<<8)|*(data_buf+17) );
//			Turn[4][1] = ( (int16)(*(data_buf+18)<<8)|*(data_buf+19) ) ;
//			Turn[4][2]	= ( (int16)(*(data_buf+20)<<8)|*(data_buf+21) );
			ANO_DT_Send_Check(*(data_buf+2),sum);
			save_pid_en = 1;
		break;
		case 0X15:
//			Turn[2][0] = ( (int16)(*(data_buf+4)<<8)|*(data_buf+5) );
//			Turn[2][1] = ( (int16)(*(data_buf+6)<<8)|*(data_buf+7) ) ;
//			Turn[2][2] = ( (int16)(*(data_buf+8)<<8)|*(data_buf+9) );
//			Turn[3][0] = ( (int16)(*(data_buf+10)<<8)|*(data_buf+11) ) ;
//			Turn[3][1] = ( (int16)(*(data_buf+12)<<8)|*(data_buf+13) );
//			Turn[3][2] = ( (int16)(*(data_buf+14)<<8)|*(data_buf+15) );
//			Turn[4][0] = ( (int16)(*(data_buf+16)<<8)|*(data_buf+17) );
//			Turn[4][1] = ( (int16)(*(data_buf+18)<<8)|*(data_buf+19) ) ;
//			Turn[4][2]	= ( (int16)(*(data_buf+20)<<8)|*(data_buf+21) );
			ANO_DT_Send_Check(*(data_buf+2),sum);
			save_pid_en = 1;
		break;
		default:break;
	
	}
}

void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Speed(float x_s,float y_s,float z_s)
{
	u8 _cnt=0;
	int16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(0.1f *x_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);

}



void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	int16 _temp;
	int32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	int16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser2(s32 bar_alt,u16 csb_alt)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(bar_alt);
	data_to_send[_cnt++]=BYTE2(bar_alt);
	data_to_send[_cnt++]=BYTE1(bar_alt);
	data_to_send[_cnt++]=BYTE0(bar_alt);

	data_to_send[_cnt++]=BYTE1(csb_alt);
	data_to_send[_cnt++]=BYTE0(csb_alt);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Power(u16 votage, u16 current, u8 flag0, u8 flag1, u16 flag2, u16 flag3)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[_cnt++]=flag0;
	data_to_send[_cnt++]=flag1;
	
	temp = flag2;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	temp = flag3;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(u8 group,s16 p1_p,s16 p1_i,s16 p1_d,s16 p2_p,s16 p2_i,s16 p2_d,s16 p3_p,s16 p3_i,s16 p3_d)
{
	u8 _cnt=0;
	int16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}


void ANO_DT_Send_User()
{
	u8 _cnt=0;
	int16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xf1; //用户数据
	data_to_send[_cnt++]=0;
	
	_temp = (int16)(imu_data.pit);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int16)(0);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int16)(0);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int16)(0);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int16)(0);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int16)(0);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

