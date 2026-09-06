#ifndef __ZPWM_H
#define __ZPWM_H

/*使用TIM3时钟，GPIOAPIN6引脚，通道一*/
void ZPWM1_Init(void);
void ZPWM1_SetCompare1(uint16_t Compare);


/*使用TIM3时钟，GPIOAPIN7引脚，通道2*/

void ZPWM2_Init(void);
void ZPWM2_SetCompare2(uint16_t Compare);

/*使用TIM3时钟，GPIOBPINB0引脚，通道3*/


void SPWM3_Init(void);
void SPWM3_SetCompare1(uint16_t Compare);
#endif
