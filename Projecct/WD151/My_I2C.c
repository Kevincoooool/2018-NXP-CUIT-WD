
/******************************* 模拟I2C函数库 *******************************/
#include "My_I2C.h"

//内部数据定义
uint8 IIC_ad_main; //器件从地址	    
uint8 IIC_ad_sub;  //器件子地址	   
uint8 *IIC_buf;    //发送|接收数据缓冲区	    
uint8 IIC_num;     //发送|接收数据个数

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC端口初始化
//  @param      NULL
//  @return     void	
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void IIC_init(void)
{
	gpio_init(My_SCL, GPO,1);
	gpio_init(My_SDA, GPO,1);
	
	port_pull (My_SCL);//ODO
	port_pull (My_SDA);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时
//  @return     void						
//  @since      v1.0
//  Sample usage:				如果IIC通讯失败可以尝试增加j的值
//-------------------------------------------------------------------------------------------------------------------
void simiic_delay(void)
{
	uint16 j=20;   
	while(j--);
}

void My_Delay_Us(uint32 us)
{
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	
}

//内部使用，用户无需调用
void IIC_start(void)
{
	DIR_OUT();
	SDA1();
	SCL1();
	My_Delay_Us(4);
	SDA0();
	My_Delay_Us(4);
	SCL0();
}

//内部使用，用户无需调用
void IIC_stop(void)
{
	DIR_OUT();
	SCL0();
	SDA0();
	My_Delay_Us(4);
	SCL1();
	SDA1();
	My_Delay_Us(4);
}

//主应答(包含ack:SDA=0和no_ack:SDA=1)
//内部使用，用户无需调用
void I2C_SendACK(unsigned char ack_dat)
{
	SCL0();
	DIR_OUT();
	if(ack_dat) 
	{
		SDA0();
	}
    else    	
	{
		SDA1();
	}
	My_Delay_Us(2);
    SCL1();
    My_Delay_Us(2);
    SCL0();
}

int SCCB_WaitAck(void)
{
	uint8 ErrorTime = 0;
	
//	DIR_OUT();
//	SDA1();
//	My_Delay_Us(1);
	DIR_IN();
    SCL1();
	My_Delay_Us(1);
	
    while (SDA)           //应答为高电平，异常，通信失败
    {
		ErrorTime++;
		if (ErrorTime > 250)
		{
			IIC_stop();
			
			return 1;
		}
    }
    SCL0();

    return 0;
}

//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
void send_ch(uint8 c)
{
	uint8 i = 8;
	
	DIR_OUT();
	SCL0();
    while(i--)
    {
        if(c & 0x80)	SDA1();//SDA 输出数据
        else			SDA0();
        c <<= 1;
		My_Delay_Us(2);
        SCL1();                //SCL 拉高，采集信号
		My_Delay_Us(2);
        SCL0();                //SCL 时钟线拉低
		My_Delay_Us(2);
    }
	SCCB_WaitAck();
}

//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|IIC_ack_main()使用
//内部使用，用户无需调用
uint8 read_ch(uint8 Ack)
{
    uint8 i, c = 0;
	
    DIR_IN();
    for(i=0;i<8;i++)
    {
        SCL0();         //置时钟线为低，准备接收数据位
        My_Delay_Us(2);
        SCL1();         //置时钟线为高，使数据线上数据有效
        c<<=1;
        if(SDA) c+=1;   //读数据位，将接收的数据存c
		My_Delay_Us(1);
    }
	if (Ack == 0)
	{
		I2C_SendACK(no_ack);
	}
	else
	{
		I2C_SendACK(ack);
	}
	
    return c;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC写数据到设备寄存器函数
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat				写入的数据
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
	IIC_start();
    send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
	send_ch( reg );   				 //发送从机寄存器地址
	send_ch( dat );   				 //发送需要写入的数据
	IIC_stop();
}
//  @brief      模拟IIC连续写数据到设备寄存器函数
uint8 simiic_write_len(uint8 dev_add, uint8 reg, uint8 len, uint8 *dat)
{
	uint8 i;
	
	IIC_start();
    send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
	send_ch( reg );   				 //发送从机寄存器地址
	for (i = 0; i < len; i++)
	{
		send_ch(dat[i]);   				 //发送需要写入的数据
	}
	IIC_stop();
	
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC从设备寄存器读取数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
uint8 simiic_read_reg(uint8 dev_add, uint8 reg, IIC_type type)
{
	uint8 dat;
	IIC_start();
    send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	send_ch( reg );   				//发送从机寄存器地址
	if(type == SCCB)IIC_stop();
	
	IIC_start();
	send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
	dat = read_ch(0);   			//发送需要写入的数据
	IIC_stop();
	
	return dat;
}
//  @brief      模拟IIC连续从设备寄存器读取数据
uint8 simiic_read_len(uint8 dev_add, uint8 reg, uint8 len, uint8 *dat)
{
	uint8 i;

	IIC_start();
    send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	send_ch( reg );   				//发送从机寄存器地址
	IIC_start();
	send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
	for (i = 0; i < len-1; i++)
	{
		dat[i] = read_ch(1);	//发送需要写入的数据
	}
	dat[i] = read_ch(0);	//发送需要写入的数据
	IIC_stop();
	
	return 0;
}

