#ifndef __DPWM_H
#define __DPWM_H


/*APB1Periph_TIM2  */
void PWM_TIM2_Common_Init(void);
void DPWM_Init(void);
void DPWM_SetCompare3(uint16_t Compare);

void DPWM2_Init(void);
void DPWM2_SetCompare4(uint16_t Compare);

#endif
