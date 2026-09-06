#ifndef __HCSR04_H
#define __HCSR04_H
#include "stm32f10x.h"     

// Device header

/*PB3
超声波模块 Trig 触发信号
推挽输出（50MHz）
HC-SR04 Trig


PB4
超声波模块 Echo 回波信号
浮空输入
HC-SR04 Echo
EXTI4_IRQHandler
EXTI_Line4（PB4）
双边沿触发（上升/下降沿）
抢占优先级0，子优先级2*/

void HC_SR04_Init(void);
uint16_t range(void);

void Timer_Init(void);

#endif
