#include "stm32f10x.h"                  // Device header
#include "ZPWM.h"
/*---------------------------------------------直流电机1--------------------------------------------*/
/**
  * @brief  直流电机1的初始化函数
  * @note   配置方向控制引脚和底层PWM
  */
void Motor1_Init(void)
{
    /* 外设时钟使能 */
    // 注意：STM32的GPIO时钟默认关闭，使用前必须手动开启
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // 使能GPIOB时钟（原为GPIOA）
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
    /* GPIO初始化结构体配置 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      // 推挽输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13; // 选择PB12和PB13引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     // 高速模式适应电机驱动需求
    GPIO_Init(GPIOB, &GPIO_InitStructure);                // 应用配置到GPIOB端口
    
    /* 特别注意：如果PB12/PB13复用为JTAG引脚（如SWD调试接口） */
    // 需要禁用JTAG功能保留SWD，添加以下代码：
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    // GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    /* PWM初始化 */
    ZPWM1_Init();  // 假设此函数配置TIM3通道1（PA6引脚）的PWM输出
}

/**
  * @brief  设置直流电机1的速度和方向
  * @param  Speed: 目标速度值，范围-100~100
  *         - 正值：正转，负值：反转，绝对值越大转速越高
  * @retval 无
  * @note   方向控制真值表：
  *         | PB12 | PB13 | 方向 |
  *         |------|------|-----|
  *         |  1   |  0   | 正转 |
  *         |  0   |  1   | 反转 |
  */
void Motor1_SetSpeed(int8_t Speed)
{
    if (Speed >= 0) {
        /* 正转模式 */
        GPIO_SetBits(GPIOB, GPIO_Pin_12);    // 设置PB12高电平
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);  // 设置PB13低电平
        
        /* PWM占空比设置 
         * 假设ZPWM1_SetCompare1的参数范围0~100对应0%~100%占空比
         * 此处直接映射速度值到PWM占空比
         */
        ZPWM1_SetCompare1(Speed);  // 设置TIM3通道1的PWM值
    } else {
        /* 反转模式 */
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);  // 设置PB12低电平
        GPIO_SetBits(GPIOB, GPIO_Pin_13);    // 设置PB13高电平
        
        /* 取反速度值传递正数给PWM模块
         * 例如：输入-75 → PWM设置为75
         */
        ZPWM1_SetCompare1(-Speed); 
    }
}


/*------------------------------------------------------------直流电机二-------------------------------------------------*/
/**
  * 函    数：直流电机初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：初始化PA8和PA9为方向控制引脚，并配置底层PWM
  */
void Motor2_Init(void)
{
    /*---------------- 开启外设时钟 ----------------*/
    // 使能GPIOA的APB2总线时钟（PA8和PA9属于APB2总线）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    /*---------------- 配置GPIO引脚 ----------------*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;   // 推挽输出模式（可直接驱动低功率设备）
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9; // 控制引脚改为PA8和PA9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 高速输出（适应PWM频率需求）
    GPIO_Init(GPIOA, &GPIO_InitStructure);              // 应用配置到GPIOA
    
    /* 初始状态：两个方向引脚均为低电平 */
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);  // PA8置低
    GPIO_ResetBits(GPIOA, GPIO_Pin_9);  // PA9置低
    
    /*---------------- 初始化PWM模块 ----------------*/
    ZPWM2_Init();  // 调用底层PWM初始化
}

/**
  * 函    数：直流电机设置速度
  * 参    数：Speed 要设置的速度，范围：-100~100
  *           正值为正转，负值为反转，绝对值越大速度越快
  * 返 回 值：无
  * 说    明：通过PA8/PA9控制方向，PWM控制速度
  */
void Motor2_SetSpeed(int8_t Speed)
{
    if (Speed >= 0) {
        /* 正转模式 */
        GPIO_SetBits(GPIOA, GPIO_Pin_8);     // PA8=1（正转使能）
        GPIO_ResetBits(GPIOA, GPIO_Pin_9);   // PA9=0
        ZPWM2_SetCompare2(Speed);            // 设置PWM占空比为Speed（0~100）
    } else {
        /* 反转模式 */
        GPIO_ResetBits(GPIOA, GPIO_Pin_8);   // PA8=0
        GPIO_SetBits(GPIOA, GPIO_Pin_9);     // PA9=1（反转使能）
        ZPWM2_SetCompare2(-Speed);           // 取反后设置PWM（确保占空比为正值）
    }
}




/*-------------------------------------------------------------直流电机三-----------------------------------------*/

/*----------------------------------------------------------
  直流电机初始化（PB5/PB6版本）
------------------------------------------------------------*/
void SMotor3_Init(void)
{
    /* 开启时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // 仅需使能GPIOB时钟（PB5/PB6无需AFIO重映射）

    /* GPIO方向控制引脚配置 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        // 推挽输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;  // PB5和PB6
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       // 高速模式
    GPIO_Init(GPIOB, &GPIO_InitStructure);                  // 初始化GPIOB

    SPWM3_Init();  // 初始化底层PWM（假设PWM输出引脚已配置，如PB0）
}

/*----------------------------------------------------------
  直流电机设置速度
  参数：Speed - 速度值，范围-100~100
------------------------------------------------------------*/
void SMotor3_SetSpeed(int8_t Speed)
{
    if (Speed >= 0) {                       // 正转方向
        GPIO_SetBits(GPIOB, GPIO_Pin_5);     // PB5置高电平
        GPIO_ResetBits(GPIOB, GPIO_Pin_6);   // PB6置低电平
        SPWM3_SetCompare1(Speed);            // 设置PWM占空比
    } else {                                 // 反转方向
        GPIO_ResetBits(GPIOB, GPIO_Pin_5);   // PB5置低电平
        GPIO_SetBits(GPIOB, GPIO_Pin_6);     // PB6置高电平
        SPWM3_SetCompare1(-Speed);           // 取反设置PWM
    }
}
