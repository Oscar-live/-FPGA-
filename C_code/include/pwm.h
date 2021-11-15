#ifndef _PWM_H_
#define _PWM_H_

#define PWM_BASE      (0x60000000)
#define PWM_DUTY_1    (PWM_BASE + (0x00))
#define PWM_DUTY_2    (PWM_BASE + (0x04))
#define PWM_DUTY_3    (PWM_BASE + (0x08))

#define PWM_REG(addr) (*((volatile uint32_t *)addr))






#endif