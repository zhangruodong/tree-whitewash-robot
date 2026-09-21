#include "stm32f10x.h"                  // Device header

/**
  * 函    数：PWM初始化
  * 参    数：无
  * 返 回 值：无
  */
  // 公共部分初始化，只需调用一次

// 公共部分初始化，只需调用一次
void PWM_TIM2_Common_Init(void)
{
    /* 开启时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);      // 开启TIM2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);      // 开启AFIO时钟（重映射需要）
    
    /* TIM2 通道映射：FullRemap -> CH1_ETR=PA15, CH2=PB3, CH3=PB10, CH4=PB11
       （PC6/PC7 是 TIM3 的完全重映射，与 TIM2 无关，别抄错）
       本文件只用 CH3/CH4 驱动两个舵机。之所以选 FullRemap 而不是 PartialRemap2：
       PartialRemap2 会把 CH1 映射到 PA0，那是水泵的脚；
       而 FullRemap 的 CH1 落在完全空闲的 PA15 上，只有 CH2 碰到 PB3（超声波 Trig）。
       且本文件只调了 TIM_OC3Init/TIM_OC4Init，CH1/CH2 输出从未使能（CC1E/CC2E=0），
       所以 PB3 实际不会被 TIM2 驱动，仍归超声波 Trig 使用。 */
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
    
    /* 时基单元初始化 */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;         // ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;         // PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    
    TIM_Cmd(TIM2, ENABLE);                                     // 启动TIM2
}

void DPWM_Init(void)
{
	 /* 开启GPIOB时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    /* 配置PB10为复用推挽输出 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 配置通道3输出比较 */
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);                    // 初始化默认值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;                        // 初始CCR值
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);                 
}

/**
  * 函    数：PWM设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~100
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void DPWM_SetCompare3(uint16_t Compare)
{
	TIM_SetCompare3(TIM2, Compare);	//设置CCR3的值（对应 PB10）
}


void DPWM2_Init(void)
{
     /* 开启GPIOB时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    /* 配置PB11为复用推挽输出 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 配置通道4输出比较 */
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;                        // 初始CCR值
    TIM_OC4Init(TIM2, &TIM_OCInitStructure); 
}

void DPWM2_SetCompare4(uint16_t Compare)
{
	  TIM_SetCompare4(TIM2, Compare);  // 设置CCR4的值，控制PB11的PWM占空比值
}


