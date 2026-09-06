#include "stm32f10x.h"     
#include "Delay.h"        

volatile uint32_t distance; // 测量距离（厘米），volatile防止编译器优化
volatile uint8_t  flag = 0; // 状态标志：0-等待回波开始 1-回波进行中
volatile uint32_t number = 0; // 定时器溢出次数（用于计算长时间回波）
volatile uint32_t times = 0;  // 总时间（溢出次数+当前计数值）


/**********************************************
 * 函数名称：HC_SR04_Init
 * 功能描述：初始化超声波模块的GPIO和外部中断
 * 参数说明：无
 **********************************************/
void HC_SR04_Init(void) {
    /*-------- 触发引脚配置（PB3：推挽输出）--------*/
	// 禁用 JTAG 但保留 SWD 调试功能（推荐）
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // 开启GPIOB时钟
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      // 推挽输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;             // 触发信号引脚PB3
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    // 高速模式确保信号陡峭
    GPIO_Init(GPIOB, &GPIO_InitStructure);               // 应用配置

    /*-------- Echo引脚配置（PB4：浮空输入）--------*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入避免干扰
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;            // 回波信号引脚PB4
    GPIO_Init(GPIOB, &GPIO_InitStructure);               // 应用配置

    /*-------- 外部中断配置（响应PB4电平变化）--------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // 必须开启AFIO时钟
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4); // 绑定PB4到EXTI4

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;           // 使用中断线4
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;            // 使能中断线
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; // 中断模式（非事件）
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // 双边沿触发
    EXTI_Init(&EXTI_InitStructure);                     // 应用配置

    /*-------- 配置EXTI4中断优先级（高优先级）--------*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);      // 使用优先级分组2
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;    // EXTI4中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;      // 使能中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 最高抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  // 子优先级2
    NVIC_Init(&NVIC_InitStructure);                     // 应用配置
}


/**********************************************
 * 函数名称：Timer_Init
 * 功能描述：配置TIM4定时器用于回波时间测量
 * 参数说明：无
 **********************************************/
void Timer_Init(void){
    /*-------- 基本定时器配置 --------*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 开启TIM4时钟
    
    TIM_TimeBaseInitTypeDef TimeBase_InitStructure;
    TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 时钟不分频
    TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TimeBase_InitStructure.TIM_Period = 1000 - 1;       // 自动重装载值（1ms周期）
    TimeBase_InitStructure.TIM_Prescaler = 72 - 1;       // 预分频72（72MHz/72=1MHz）
    TimeBase_InitStructure.TIM_RepetitionCounter = 0;    // 高级定时器专用，设为0
    TIM_TimeBaseInit(TIM4, &TimeBase_InitStructure);     // 应用配置

    /*-------- 中断相关配置 --------*/
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);           // 使能更新中断
    TIM_InternalClockConfig(TIM4);                      // 明确使用内部时钟

    /*-------- 配置TIM4中断优先级 --------*/
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;      // TIM4中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;   // 子优先级1
    NVIC_Init(&NVIC_InitStructure);                     // 应用配置
}


/**********************************************
 * 函数名称：range
 * 功能描述：触发超声波模块并计算平均距离
 * 返回值：uint16_t 测量距离（厘米）
 **********************************************/
uint16_t range(void){
    int temp = 0;
    distance = 0; // 清空累计距离
    
    // 连续测量10次取平均
    for(int i=0; i<10; ++i){
        // 发送10us高电平触发信号（实际15us考虑函数调用时间）
        GPIO_SetBits(GPIOB, GPIO_Pin_3);  // PB3输出高电平
        Delay_us(15);                     // 维持高电平
        GPIO_ResetBits(GPIOB, GPIO_Pin_3);// PB3输出低电平
        Delay_ms(65);                     // 等待回波稳定（模块要求>60ms）
        
        // 累加计算时间对应的距离（声速343m/s = 0.0343cm/μs）
        distance += (times * 0.0343 / 2); // 除以2（往返时间）
    }
    temp = distance / 10; // 计算10次平均值
    return temp;          // 返回最终结果
}


/**********************************************
 * 中断服务函数：TIM4_IRQHandler
 * 功能描述：处理TIM4的溢出中断
 **********************************************/
void TIM4_IRQHandler(void) {
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET){
        number++; // 溢出次数+1（每次溢出代表1ms）
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update); // 必须清除中断标志
    }
}


/**********************************************
 * 中断服务函数：EXTI4_IRQHandler
 * 功能描述：处理PB4引脚的双边沿中断
 **********************************************/
void EXTI4_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line4) == SET){
        if(flag == 0){ // 上升沿：回波开始
            number = 0;     // 重置溢出计数器
            flag = 1;       // 标记测量开始
            TIM_SetCounter(TIM4, 0); // 定时器计数器归零
            TIM_Cmd(TIM4, ENABLE);  // 启动定时器
        }
        else {          // 下降沿：回波结束
            TIM_Cmd(TIM4, DISABLE); // 停止定时器
            flag = 0;
            // 计算总时间 = 溢出次数*1000μs + 当前计数值（单位：μs）
            times = number * 1000 + TIM_GetCounter(TIM4); 
        }
        EXTI_ClearITPendingBit(EXTI_Line4); // 必须清除中断标志
    }
}
