#ifndef __HCSR04_H
#define __HCSR04_H
#include "stm32f10x.h"

/* HC-SR04 ultrasonic module
   Trig = PB3 (output), Echo = PB4 (input, EXTI4 both edges)
   TIM4 (1 MHz / 1 ms) times the echo pulse; times (us) set in ISR. */

void HC_SR04_Init(void);
void Timer_Init(void);

uint8_t  US_Update(void);      /* advance measurement, returns 1 when result ready */
uint16_t US_GetDistance(void); /* last averaged distance in cm */

#endif
