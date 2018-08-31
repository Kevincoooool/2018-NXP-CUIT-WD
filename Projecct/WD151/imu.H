#ifndef __IMU_H__
#define __IMU_H__



#define RtA 		  57.324841				//弧度到角度
#define AtR    		0.0174533				//度到角度
#define Acc_G 		0.0000610351		//加速度变成G
#define Gyro_G 	0.0610351				  //角速度变成度   此参数对应陀螺2000度每秒
#define Gyro_Gr	0.0010653				  //角速度变成弧度	此参数对应陀螺2000度每秒


extern float yaw;
extern float pitch;
extern float roll;				
		//四元数计算出的角度


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);



#endif
