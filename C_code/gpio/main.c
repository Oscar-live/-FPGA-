#include <stdint.h>

#include "../include/gpio.h"
#include "../include/utils.h"
#include "../include/timer.h"

void gpio_init(void)
{
	gpio_mode_clear;
    // gpio0_mode_out;  // gpio0输出模式
    // gpio1_mode_out;  // gpio1输出模式
	// gpio2_mode_out;	 // gpio2输出模式
	gpio_data_reset; // gpio初始全部置0	
}

void timer_init(uint32_t period)
{
	TIMER0_REG(TIMER0_VALUE) = period;  // period/50M s period
    TIMER0_REG(TIMER0_CTRL) = 0x07;     // enable interrupt and start timer	

}

void delay_ms(uint32_t delay) //适用于系统时钟为50MHz
{
	uint32_t delay_index = delay;
	for(delay_index; delay_index > 0; delay_index--);
}

static volatile uint32_t count;

int main()
{
	gpio_init(); //GPIO初始化 清零
	timer_init(500); // 500000 10ms period   //500 10us
	count = 0;
    gpio0_mode_out;  // gpio0输出模式
	gpio1_mode_out;  // gpio1输出模式
    while (1) {
        // 定时器500ms，每次触发一次GPIO信号反转
        if (count == 2) {//20us
            count = 0;
            GPIO_REG(GPIO_DATA) ^= 0x01; //
        }
    }
    return 0;
}

void timer0_irq_handler()
{
    TIMER0_REG(TIMER0_CTRL) |= (1 << 2) | (1 << 0);  // clear int pending and start timer

    count++;
}
