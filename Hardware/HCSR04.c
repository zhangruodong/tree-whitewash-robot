#include "stm32f10x.h"
#include "Delay.h"

/* Non-blocking HC-SR04 driver.
   US_Update() is called once per main loop and drives a small state machine:
   fire trigger -> wait echo (or timeout) -> cool-down -> repeat.
   After US_SAMPLE_N samples it returns 1 with a fresh averaged distance
   available from US_GetDistance(). No Delay_ms() blocking, so the T/G
   commands on USART2 stay responsive. */

#define US_SAMPLE_N        10    /* samples per averaged result            */
#define US_ECHO_TIMEOUT_MS 40    /* give up waiting for echo after 40 ms   */
#define US_CYCLE_MS        60    /* min gap between triggers (>60 ms)      */
#define US_MAX_DIST_CM     400   /* clamp value when no echo (out of range)*/

extern volatile uint32_t timer_counter;   /* 1 ms tick, defined in system.c */

/* ISR state */
volatile uint32_t number = 0;             /* TIM4 overflow count (1 ms each) */
volatile uint8_t  flag = 0;               /* 0 = waiting rising, 1 = echo high */
volatile uint32_t times = 0;              /* echo width in us (set on falling) */
volatile uint8_t  echo_done = 0;          /* set when a falling edge ends echo */

/* sample state machine */
static uint8_t  us_phase = 0;
static uint32_t us_phase_start = 0;
static uint32_t us_sum = 0;
static uint32_t us_count = 0;
static uint32_t us_result = US_MAX_DIST_CM;  /* "far" until first real reading */

void HC_SR04_Init(void) {
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      /* Trig = PB3 out */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; /* Echo = PB4 in  */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
}

void Timer_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TimeBase_InitStructure;
    TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TimeBase_InitStructure.TIM_Period = 1000 - 1;    /* 1 ms period */
    TimeBase_InitStructure.TIM_Prescaler = 72 - 1;   /* 1 MHz clock */
    TimeBase_InitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TimeBase_InitStructure);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_InternalClockConfig(TIM4);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/* Advance one step of the measurement. Returns 1 when a fresh averaged
   result is ready (read it with US_GetDistance()). Call every loop. */
uint8_t US_Update(void) {
    switch (us_phase) {
    case 0:                                  /* fire trigger              */
        echo_done = 0;
        times = 0;
        GPIO_SetBits(GPIOB, GPIO_Pin_3);
        Delay_us(15);                        /* ~15 us trigger pulse      */
        GPIO_ResetBits(GPIOB, GPIO_Pin_3);
        us_phase_start = timer_counter;
        us_phase = 1;
        break;

    case 1:                                  /* wait echo or timeout      */
        if (echo_done) {
            us_sum += (uint32_t)(times * 343UL / 20000UL);  /* us -> cm (/2) */
            us_count++;
            us_phase = 2;
            us_phase_start = timer_counter;
        } else if (timer_counter - us_phase_start >= US_ECHO_TIMEOUT_MS) {
            us_sum += US_MAX_DIST_CM;        /* no echo -> treat as far   */
            us_count++;
            us_phase = 2;
            us_phase_start = timer_counter;
        }
        break;

    case 2:                                  /* cool-down before next     */
        if (timer_counter - us_phase_start >= US_CYCLE_MS) {
            if (us_count >= US_SAMPLE_N) {
                us_result = us_sum / us_count;
                us_sum = 0;
                us_count = 0;
                us_phase = 0;
                return 1;                    /* fresh averaged result     */
            }
            us_phase = 0;
        }
        break;
    }
    return 0;
}

uint16_t US_GetDistance(void) {
    return (uint16_t)us_result;
}

void TIM4_IRQHandler(void) {
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) {
        number++;                            /* each overflow = 1 ms      */
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}

void EXTI4_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line4) == SET) {
        if (flag == 0) {                     /* rising edge: start timing */
            number = 0;
            flag = 1;
            TIM_SetCounter(TIM4, 0);
            TIM_Cmd(TIM4, ENABLE);
        } else {                             /* falling edge: stop, read  */
            TIM_Cmd(TIM4, DISABLE);
            flag = 0;
            times = number * 1000 + TIM_GetCounter(TIM4);   /* us */
            echo_done = 1;
        }
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}
