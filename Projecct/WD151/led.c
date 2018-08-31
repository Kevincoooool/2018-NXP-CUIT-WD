#include "led.h"
/******************************************************************************************************************
2018年全国大学生恩智浦杯智能车竞赛
成都信息工程大学   成信WD队
作者QQ：97354734
文件作用:
*******************************************************************************************************************/
void led_init(void)
{
	gpio_init(I4,GPO,1);                         //初始化LED0  ，灭    
	gpio_init(I1,GPO,1);                         //初始化LED0  ，灭    
	gpio_init(I0,GPO,1);                         //初始化LED0  ，灭       
}
void button_init(void)
{
  gpio_init (BUTTON_UP, GPI,1);
  gpio_init (BUTTON_LEFT, GPI,1);
  gpio_init (BUTTON_RIGHT, GPI,1);
  gpio_init (BUTTON_MID, GPI,1);
  gpio_init (BUTTON_DOWN, GPI,1);
	port_pull(BUTTON_UP);
	port_pull(BUTTON_LEFT);
	port_pull(BUTTON_RIGHT);
	port_pull(BUTTON_MID);
	port_pull(BUTTON_DOWN);

}
