#ifndef _GPIO_H_
#define _GPIO_H_

#define GPIO_BASE      (0x40000000)
#define GPIO_CTRL      (GPIO_BASE + (0x00))
#define GPIO_DATA      (GPIO_BASE + (0x04))

#define GPIO_REG(addr) (*((volatile uint32_t *)addr))

//GPIO_mode_reset
#define gpio_mode_clear ((*((volatile uint32_t *)GPIO_CTRL)) &= 0x0)
#define gpio_data_reset ((*((volatile uint32_t *)GPIO_DATA)) &= 0x0)
#define gpio_data_set   ((*((volatile uint32_t *)GPIO_DATA)) &= 0xffff)

//GPIO_out_mode
#define gpio0_mode_out ((*((volatile uint32_t *)GPIO_CTRL)) |= 0x1)
#define gpio1_mode_out ((*((volatile uint32_t *)GPIO_CTRL)) |= 0x1<<2)
#define gpio2_mode_out ((*((volatile uint32_t *)GPIO_CTRL)) |= 0x1<<4)
#define gpio3_mode_out ((*((volatile uint32_t *)GPIO_CTRL)) |= 0x1<<6)
#define gpio4_mode_out ((*((volatile uint32_t *)GPIO_CTRL)) |= 0x1<<8)
#define gpio5_mode_out ((*((volatile uint32_t *)GPIO_CTRL)) |= 0x1<<10)
#define gpio6_mode_out ((*((volatile uint32_t *)GPIO_CTRL)) |= 0x1<<12)
#define gpio7_mode_out ((*((volatile uint32_t *)GPIO_CTRL)) |= 0x1<<14)
#define gpio8_mode_out ((*((volatile uint32_t *)GPIO_CTRL)) |= 0x1<<16)
#define gpio9_mode_out ((*((volatile uint32_t *)GPIO_CTRL)) |= 0x1<<18)

//GPIO_in_mode
#define gpio0_mode_in ((*((volatile uint32_t *)GPIO_DATA)) |= 0x1<<1)
#define gpio1_mode_in ((*((volatile uint32_t *)GPIO_DATA)) |= 0x1<<3)
#define gpio2_mode_in ((*((volatile uint32_t *)GPIO_DATA)) |= 0x1<<5)
#define gpio3_mode_in ((*((volatile uint32_t *)GPIO_DATA)) |= 0x1<<7)
#define gpio4_mode_in ((*((volatile uint32_t *)GPIO_DATA)) |= 0x1<<9)
#define gpio5_mode_in ((*((volatile uint32_t *)GPIO_DATA)) |= 0x1<<11)
#define gpio6_mode_in ((*((volatile uint32_t *)GPIO_DATA)) |= 0x1<<13)
#define gpio7_mode_in ((*((volatile uint32_t *)GPIO_DATA)) |= 0x1<<15)
#define gpio8_mode_in ((*((volatile uint32_t *)GPIO_DATA)) |= 0x1<<17)
#define gpio9_mode_in ((*((volatile uint32_t *)GPIO_DATA)) |= 0x1<<19)


void forward(int pwm_left,int pwm_right);
void back(int pwm);
void turn_right(int pwm);
void turn_left(int pwm);
void gpio_init(void);


#endif
