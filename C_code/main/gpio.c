#include "../include/gpio.h"
#include "../include/pwm.h"

void forward(int pwm_left,int pwm_right)
{
	PWM_REG(PWM_DUTY_1) = pwm_left;
	
	PWM_REG(PWM_DUTY_2) = pwm_right;
	
	GPIO_REG(GPIO_DATA) |= 0b1001;
}

void back(int pwm)
{
	PWM_REG(PWM_DUTY_1) = pwm;
	PWM_REG(PWM_DUTY_2) = pwm;
	
	GPIO_REG(GPIO_DATA) |= 0b0110;
}

void turn_right(int pwm)
{
	PWM_REG(PWM_DUTY_1) = pwm;
	PWM_REG(PWM_DUTY_2) = pwm;
	
	GPIO_REG(GPIO_DATA) |= 0b1010;
}


void turn_left(int pwm)
{
	PWM_REG(PWM_DUTY_1) = pwm;
	PWM_REG(PWM_DUTY_2) = pwm;
	
	GPIO_REG(GPIO_DATA) |= 0b0101;
}


