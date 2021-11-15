#include <stdint.h>

#include "../include/gpio.h"
#include "gpio.c"
#include "../include/utils.h"
#include "../include/timer.h"
#include "../include/pwm.h"

void gpio_init()
{
	gpio_mode_clear; // gpio_mode清零
	gpio_data_reset; // gpio初始全部置0
	//set mode
    gpio0_mode_out;  // gpio0输出模式
    gpio1_mode_out;  // gpio1输出模式
	gpio2_mode_out;	 // gpio2输出模式
	gpio3_mode_out;
	gpio4_mode_out;
	gpio5_mode_out;
	gpio6_mode_out;
	gpio7_mode_out;
	//set data
	//GPIO_REG(GPIO_DATA) |= 0b00000000; //gpio0\gpio1\gpio2...gpio7初始化设置为高电平
}

void pwm_init()
{
    //周期设定值是1000
    PWM_REG(PWM_DUTY_1) = 300;//占空比设为
	PWM_REG(PWM_DUTY_2) = 400;//占空比设为
	PWM_REG(PWM_DUTY_3) = 500;//占空比设为
}


//变量定义一定要定义在main外边
int main()
{    
	//GPIO配置
    gpio_init();
	//高电平占空比为 pwm
	//forward(400,400); //left,right
	PWM_REG(PWM_DUTY_1) = 250;
	PWM_REG(PWM_DUTY_2) = 810;
	GPIO_REG(GPIO_DATA) |= 0b1001;
	
	
	return 0;
}




//int i = 0;
//char str_buf[4] = {0,0,0,0};
//int main()
//{
//	data_str = receive(data); //data = 形如 “2E” 的16进制字符
//	str_buf[i] = data_str;
//	i = i + 1;
//	if(i = 3) //接收四次为一个循环
//	{
//		i = 0;
//		Tem_num = str_to_10(str[0:1]); //前两个为温度
//		display(num_10);
//		Hum_num = str_to_10(str[2:3]);//后两个为湿度
//		display(num_10);
//	}
//}








