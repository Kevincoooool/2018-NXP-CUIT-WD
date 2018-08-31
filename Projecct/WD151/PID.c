#include "PID.h"
#include "control.h"
/******************************************************************************************************************
2018年全国大学生恩智浦杯智能车竞赛
成都信息工程大学   成信WD队
作者QQ：97354734
文件作用:PID函数及参数设置
*******************************************************************************************************************/
PID Speed_PID, Angle_PID, Ang_gyro_PID, Turn_gyro_PID, Turn_PID, Distance_PID;	//定义PID参数结构体
float Speed[4]    = {20,   0,    4, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]    = {0.3, 0, 0.06, 500};		// 角度环PID
float Ang_gyro[5] = {0.33, 0.028,    0, 1000, 0};		// 角速度环PID
float Turn_gyro[5]= {0.022,  0.0, 0.05,   70, 0};	// 转向环PID 位置	0.017	0.02
float Turn[5][4]  = {{100, 7, 8, 400},	// 130	// 转向外环动态PID	中线法
					{100, 7, 8, 400},	// 140
					{100, 8, 9, 400},	// 150
					{110, 8, 9, 400},	// 160
					{110, 9, 10, 400}};	// 170

// PID参数初始化
void PID_Parameter_Init(PID *sptr)
{
	sptr->SumError  = 0;
	sptr->LastError = 0;	
	sptr->PrevError = 0;	
	sptr->LastData  = 0;
}

// 位置式动态PID控制
int32 PlacePID_Control(PID *sprt, float *PID, float NowPiont, float SetPoint)
{
	//定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	float iError;	//当前误差
	int  Actual;	//最后得出的实际输出值
	float Kp;		//动态P
	
	iError = SetPoint - NowPiont;	//计算当前误差
	sprt->SumError += iError*0.01;
	if (sprt->SumError >= PID[KT])
	{
		sprt->SumError = PID[KT];
	}
	else if (sprt->SumError <= PID[KT])
	{
		sprt->SumError = -PID[KT];
	}
	//二次函数是为了达到 误差越大  反应越快 回复力越大 其中 KI值是误差为0时的P 也就是直道上的P值
	Kp = 1.0 * (iError*iError) / PID[KP] + PID[KI];	//P值与差值成二次函数关系，此处P和I不是PID参数，而是动态PID参数，要注意！！！
	
	Actual = Kp * iError
		   + PID[KD] * ((0.8*iError + 0.2*sprt->LastError) - sprt->LastError);//不完全微分  
	sprt->LastError = iError;		//更新上次误差

	Actual = range_protect(Actual, -260, 260);

	return Actual;
}

// 位置式PID控制
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	int32 Realize;	// 最后得出的实际增量

	sptr->Dis_Err = Point - NowData;	// 计算当前误差
	sptr->SumError += PID[KI] * sptr->Dis_Err;	// 误差积分
	
	if (sptr->SumError >= PID[KT])
	{
		sptr->SumError = PID[KT];
	}
	else if (sptr->SumError <= -PID[KT])
	{
		sptr->SumError = -PID[KT];
	}
	
	Realize = PID[KP] * sptr->Dis_Err
			+ sptr->SumError
			+ PID[KD] *(sptr->Dis_Err  - sptr->LastError);
//			+ PID[KB] * ( NowData- sptr->LastData); //微分先行
	
	sptr->PrevError = sptr->LastError;	// 更新前次误差
	sptr->LastError = sptr->Dis_Err;		  	// 更新上次误差
	sptr->LastData  = NowData;			// 更新上次数据

	return Realize;	// 返回实际值
}

// 增量式PID电机控制
int32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	int32 iError,	//当前误差
		Increase;	//最后得出的实际增量

	iError = Point - NowData;	// 计算当前误差

	Increase =  PID[KP] * (iError - sptr->LastError)
			  + PID[KI] * iError
			  + PID[KD] * (iError - 2 * sptr->LastError + sptr->PrevError);
	
	sptr->PrevError = sptr->LastError;	// 更新前次误差
	sptr->LastError = iError;		  	// 更新上次误差
	sptr->LastData  = NowData;			// 更新上次数据
	
	return Increase;	// 返回增量
}
/* 5.6
float MOTOR[4]   = {70,   0,    3, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]   = {0.33, 0, 0.03, 500};		// 角度环PID
float Ang_Vel[4] = {0.25, 0.06, 0, 1000};		// 角速度环PID
float Direct[4]  = {0.35, 0.0, 0.01, 70};	// 转向环PID 位置	0.017	0.02
float Turn[5][4] = {{100, 8, 8, 400},	// 130	// 转向外环动态PID	中线法
										{100, 9, 8, 400},	// 140
										{100, 10, 8, 400},	// 150
										{100, 13, 8, 400},	// 160
										{100, 15, 8, 400}};	// 170
5-7
float MOTOR[4]   = {35,   0.001,    0, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]   = {0.32, 0, 0.02, 500};		// 角度环PID
float Ang_Vel[4] = {0.22, 0.05, 0, 1000};		// 角速度环PID
float Direct[4]  = {0.058, 0.0, 0.04, 70};	// 转向环PID 位置	0.017	0.02
float Turn[5][4] = {{100, 8, 8, 400},	// 130	// 转向外环动态PID	中线法
										{100, 9, 7, 400},	// 140
										{100, 11, 6, 400},	// 150
										{100, 14, 4, 400},	// 160
										{100, 17, 2, 400}};	// 170
5-11
float MOTOR[4]   = {25,   0.0001,    0, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]   = {0.3, 0, 0.01, 500};		// 角度环PID
float Ang_Vel[4] = {0.28, 0.04, 0, 1000};		// 角速度环PID
float Direct[4]  = {0.095, 0.0000001, 0.08, 70};	// 转向环PID 位置	0.017	0.02
float Turn[5][4] = {{100, 8, 8, 400},	// 130	// 转向外环动态PID	中线法
										{120, 8, 8, 400},	// 140
										{350, 9, 9, 400},	// 150
										{650, 11, 11, 400},	// 160
										{850, 13, 12, 400}};	// 170
5-12 下午
float MOTOR[4]   = {25,   0.0001,    0, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]   = {0.3, 0, 0.01, 500};		// 角度环PID
float Ang_Vel[4] = {0.25, 0.05, 0, 1000};		// 角速度环PID
float Direct[4]  = {0.095, 0.0001, 0.9, 70};	// 转向环PID 位置	0.017	0.02
float Turn[5][4] = {{60, 1, 3, 400},	// 130	// 转向外环动态PID	中线法
										{60, 1.5, 3, 400},	// 140
										{60, 2, 3, 400},	// 1if 
										{60, 3, 4, 400},	// 160
										{60, 3.5, 5, 400}};	// 170
										float MOTOR[4]   = {25,   0.0001,    0, 1000};		// 速度环PID	最后一项为积分限幅
5-13 晚
float Angle[4]   = {0.3, 0, 0.01, 500};		// 角度环PID
float Ang_Vel[4] = {0.25, 0.05, 0, 1000};		// 角速度环PID
float Direct[4]  = {0.11, 0.00001, 0.9, 70};	// 转向环PID 位置	0.017	0.02
float Turn[5][4] = {{60, 1, 3, 400},	// 130	// 转向外环动态PID	中线法
										{60, 1.5, 3, 400},	// 140
										{60, 1.8, 4, 400},	// 150
										{60, 2.0, 5, 400},	// 160
										{65, 2.1, 5, 400}};	// 170
5-16
float MOTOR[4]   = {25,   0.001,    0, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]   = {0.3, 0, 0.02, 500};		// 角度环PID
float Ang_Vel[4] = {0.3, 0.05, 0, 1000};		// 角速度环PID
float Direct[4]  = {0.28, 0.0001, 0.08, 70};	// 转向环PID 位置	0.017	0.02
float Turn[5][4] = {{50, 6, 6, 400},	// 130	// 转向外环动态PID	中线法
										{50, 6, 6, 400},	// 140
										{50, 6, 6, 400},	// 150
										{50, 6, 6, 400},	// 160
										{50, 6, 7, 400}};	// 170
5-20 省赛2.54
float MOTOR[4]   = {30,   0,    3, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]   = {0.3, 0, 0.05, 500};		// 角度环PID
float Ang_Vel[4] = {0.3, 0.04, 0, 1000};		// 角速度环PID
float Direct[4]  = {0.024, 0.0001, 0.0265, 70};	// 转向环PID 位置	0.017	0.02
float Turn[5][4] = {{100, 6, 8, 400},	// 130	// 转向外环动态PID	中线法
										{100, 6, 8, 400},	// 140
										{100, 6, 8, 400},	// 150
										{110, 7, 9, 400},	// 160
										{110, 8, 9, 400}};	// 170
6-25
float Speed[4]    = {10,   0,    3, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]    = {0.3, 0, 0.1, 500};		// 角度环PID
float Ang_gyro[4] = {0.5, 0.035, 0, 1000};		// 角速度环PID
float Turn_gyro[4]= {0.028, 0, 0.03, 70};	// 转向环PID 位置	0.017	0.02
float Turn[5][4]  = {{100, 7, 12, 400},	// 130	// 转向外环动态PID	中线法
										{100, 7, 13, 400},	// 140
										{100, 8, 14, 400},	// 150
										{110, 8, 14, 400},	// 160
										{110, 10, 15, 400}};	// 170
7-5  D
float Speed[4]    = {20,   0,    5, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]    = {0.3, 0, 0.05, 500};		// 角度环PID
float Ang_gyro[4] = {0.3, 0.023, 0, 1000};		// 角速度环PID
float Turn_gyro[4]= {0.023, 0, 0.035, 70};	// 转向环PID 位置	0.017	0.02
float Turn[5][4]  = {{100, 6, 8, 400},	// 130	// 转向外环动态PID	中线法
										{100, 7, 8, 400},	// 140
										{100, 8, 8, 400},	// 150
										{100, 8, 8, 400},	// 160
										{100, 9, 8, 400}};	// 170
7-9
float Speed[4]    = {20,   0,    5, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]    = {0.3, 0, 0.05, 500};		// 角度环PID
float Ang_gyro[5] = {0.3, 0.025,    0, 1000, 0};		// 角速度环PID
float Turn_gyro[5]= {0.018,    0, 0.04,   70, 0};	// 转向环PID 位置	0.017	0.02
float Turn[5][4]  = {{100, 9, 10, 400},	// 130	// 转向外环动态PID	中线法
										{100, 9, 10, 400},	// 140
										{100, 9, 10, 400},	// 150
										{100, 9, 12, 400},	// 160
										{100, 9, 12, 400}};	// 170
8-11
float Speed[4]    = {25,   0,    4, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]    = {0.3, 0, 0.1, 500};		// 角度环PID
float Ang_gyro[5] = {0.4, 0.032,    0, 1000, 0};		// 角速度环PID
float Turn_gyro[5]= {0.02,    0, 0.045,   70, 0};	// 转向环PID 位置	0.017	0.02
float Turn[5][4]  = {{100, 8, 8, 400},	// 130	// 转向外环动态PID	中线法
					{100, 8, 9, 400},	// 140
					{100, 9, 9, 400},	// 150
					{110, 9, 10, 400},	// 160
					{110, 10, 10, 400}};	// 170
8-12
float Speed[4]    = {25,   0,    4, 1000};		// 速度环PID	最后一项为积分限幅
float Angle[4]    = {0.3, 0, 0.1, 500};		// 角度环PID
float Ang_gyro[5] = {0.42, 0.03,    0, 1000, 0};		// 角速度环PID
float Turn_gyro[5]= {0.02,    0, 0.06,   70, 0};	// 转向环PID 位置	0.017	0.02
float Turn[5][4]  = {{100, 8, 8, 400},	// 130	// 转向外环动态PID	中线法
					{100, 8, 9, 400},	// 140
					{100, 9, 9, 400},	// 150
					{110, 9, 10, 400},	// 160
					{110, 10, 10, 400}};	// 170
*/

