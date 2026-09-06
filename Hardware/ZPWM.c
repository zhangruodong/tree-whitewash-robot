#include "stm32f10x.h"                  // Device header

/**
  * 函    数：PWM初始化
  * 参    数：无
  * 返 回 值：无
  */
void ZPWM1_Init(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);			//开启TIM3的时钟，TIM3属于APB1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//开启GPIOA的时钟
	
	/*GPIO初始化*/
	/*配置PA6为复用推挽输出（TIM3_CH1）*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);							//将PA6引脚初始化为复用推挽输出	
																	//受外设控制的引脚，均需要配置为复用模式
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIM3);		//选择TIM3为内部时钟，若不调用此函数，TIM默认也为内部时钟
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;                 //计数周期，即ARR的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1;               //预分频器，即PSC的值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器才会用到
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM2的时基单元
	
	/*输出比较初始化*/ 
	TIM_OCInitTypeDef TIM_OCInitStructure;							//定义结构体变量
	TIM_OCStructInit(&TIM_OCInitStructure);                         //结构体初始化，若结构体没有完整赋值
	                                                                //则最好执行此函数，给结构体所有成员都赋一个默认值
	                                                                //避免结构体初值不确定的问题
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               //输出比较模式，选择PWM模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //输出极性，选择为高，若选择极性为低，则输出高低电平取反
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;								//初始的CCR值
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);                        //将结构体变量交给TIM_OC3Init，配置TIM2的输出比较通道3
	
	/*TIM使能*/
	TIM_Cmd(TIM3, ENABLE);			//使能TIM2，定时器开始运行
}

/**
  * 函    数：PWM设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~100
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void ZPWM1_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare1(TIM3, Compare);		//设置CCR3的值
}

/**
  * @brief  PWM初始化函数，配置TIM3通道2（PA7引脚）输出PWM
  * @param  无
  * @retval 无
  */
void ZPWM2_Init(void)
{
    /*------------------------ 时钟配置 ------------------------*/
    // 使能TIM3的APB1总线时钟（定时器属于低速外设）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    // 使能GPIOA的APB2总线时钟（GPIO属于高速外设）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /*------------------------ GPIO配置 ------------------------*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;  // 复用推挽模式（PWM信号需要复用功能）
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;        // 使用PA7引脚（TIM3_CH2的复用功能引脚）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // 引脚输出速度50MHz（与PWM频率匹配）
    GPIO_Init(GPIOA, &GPIO_InitStructure);             // 初始化GPIOA

    /*--------------------- 定时器基础配置 ---------------------*/
    TIM_InternalClockConfig(TIM3);  // 选择TIM3的内部时钟源（默认为APB1时钟）
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 时钟分频（不分频）
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseInitStructure.TIM_Period        = 100 - 1;          // 自动重装载值（决定PWM周期）
    TIM_TimeBaseInitStructure.TIM_Prescaler     = 36 - 1;           // 预分频器（与时钟频率共同决定PWM频率）
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            // 重复计数器（高级定时器专用，此处设为0）
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);             // 应用定时器配置

    /*--------------------- PWM通道配置 ----------------------*/
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);  // 初始化结构体为默认值
    
    // PWM模式1：计数器 < CCRx时输出有效电平
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;    
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High; // 有效电平为高电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 使能输出
    TIM_OCInitStructure.TIM_Pulse       = 0;                   // 初始占空比为0
    
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  // 配置TIM3通道2（CH2对应PA7）
    
    TIM_Cmd(TIM3, ENABLE);  // 启动定时器计数器
}

/**
  * @brief  设置PWM占空比
  * @param  Compare: 比较值（范围0~TIM_Period）
  * @retval 无
  * @note   占空比 = Compare / (TIM_Period + 1)
  *         例如：当TIM_Period=99，Compare=50时，占空比为50%
  */
void ZPWM2_SetCompare2(uint16_t Compare)
{
    // 设置TIM3通道2的比较寄存器值（直接控制占空比）
    TIM_SetCompare2(TIM3, Compare);
}


/*----------------------------------------------------------
  PWM初始化函数
  配置TIM3通道3输出PWM，对应引脚PB0
------------------------------------------------------------*/
void SPWM3_Init(void)
{
    /* 时钟配置 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   // 使能TIM3时钟（APB1总线）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // 使能GPIOB时钟（APB2总线）

    /* GPIO配置 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      // 复用推挽输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;            // 选择PB0引脚（TIM3_CH3）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    // IO口速度为50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);               // 应用配置到GPIOB

    /* 定时器时基单元配置 */
    TIM_InternalClockConfig(TIM3);                       // 使用内部时钟源
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;   // 时钟分频系数1
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;      // 自动重装载值（决定PWM周期）
    TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1;    // 预分频系数（72MHz主频下，72/36=2MHz）
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;  // 重复计数器（高级定时器使用）
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);   // 应用时基配置

    /* PWM通道配置 */
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);              // 用默认参数初始化结构体
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;    // PWM模式1（CNT<CCR时有效）
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出极性高（有效电平为高）
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 使能输出
    TIM_OCInitStructure.TIM_Pulse = 0;                   // 初始占空比为0（CCR=0）
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);             // 配置通道3（TIM3_CH3对应PB0）

    TIM_Cmd(TIM3, ENABLE);                               // 启动定时器
}

/*----------------------------------------------------------
  设置PWM占空比
  参数：Compare - 比较值（范围0~TIM_Period）
------------------------------------------------------------*/
void SPWM3_SetCompare1(uint16_t Compare)
{
    TIM_SetCompare3(TIM3, Compare);  // 修改通道3的比较寄存器值（直接影响占空比）
}

