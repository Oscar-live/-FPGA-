#ifndef _TIMER_H_
#define _TIMER_H_

#define TIMER0_BASE   (0x20000000)
#define TIMER0_CTRL   (TIMER0_BASE + (0x00))
#define TIMER0_COUNT  (TIMER0_BASE + (0x04))
#define TIMER0_VALUE  (TIMER0_BASE + (0x08))
#define TIMER0_I  	  (TIMER0_BASE + (0x0C))
#define TIMER0_SUM    (TIMER0_BASE + (0x10))


#define TIMER0_REG(addr) (*((volatile uint32_t *)addr))

void timer_init(uint32_t period);

#endif
