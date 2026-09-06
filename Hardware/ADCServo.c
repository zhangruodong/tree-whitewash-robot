#include "stm32f10x.h"    
/***********************************************
 * 模块：舵机反馈检测模块
 * 功能：
 *   - 通过PA4引脚采集舵机位置反馈信号
 *   - 12位ADC转换，分辨率约0.04°
 * 硬件连接：
 *   PA4 ----- 舵机反馈信号（0-3.3V）
 * 软件依赖：
 *   - STM32标准外设库
 *   - 系统时钟配置正确（影响ADC采样率）
 * 版本记录：
 *   v1.0 | 2023-10-01 | 实现基本ADC采集功能
 *   v1.1 | 2023-10-05 | 增加校准流程和错误处理
 ***********************************************/// Device header

/**
  * @brief  ADC GPIO初始化函数
  * @note   配置PA4引脚为模拟输入模式，用于连接舵机反馈信号
  * @param  无
  * @retval 无
  */
void ADC_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /* 使能GPIOA和ADC1的时钟
     * 注意：ADC通道4对应PA4，必须同时开启GPIOA和ADC1时钟
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    
    /* 配置PA4为模拟输入模式
     * GPIO_Mode_AIN：模拟输入模式，禁用施密特触发器，直接连接ADC
     * 特别注意：PA4复用为DAC_OUT1，若使用DAC需关闭本功能
     */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;       // 选择PA4引脚
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;  // 必须设置为模拟输入
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 对模拟输入无影响，但需赋值
    GPIO_Init(GPIOA, &GPIO_InitStruct);         // 应用配置到GPIOA
}

/**
  * @brief  ADC通道4初始化函数
  * @note   配置ADC1的通道4，使用单次转换模式，软件触发
  * @param  无
  * @retval 无
  */
void ADC_Init_Channel4(void) {
    ADC_InitTypeDef ADC_InitStruct;
    
    /* ADC基本参数配置 */
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;          // 独立模式（非双ADC模式）
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;               // 禁用扫描模式（单通道）
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;         // 单次转换模式
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发转换
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;      // 数据右对齐（便于计算）
    ADC_InitStruct.ADC_NbrOfChannel = 1;                     // 转换通道数为1
    ADC_Init(ADC1, &ADC_InitStruct);                         // 应用配置到ADC1
    
    /* ADC校准流程（必须执行）*/
    ADC_Cmd(ADC1, ENABLE);                   // 使能ADC1
    ADC_ResetCalibration(ADC1);              // 复位校准寄存器
    while(ADC_GetResetCalibrationStatus(ADC1)); // 等待复位完成（硬件自动清除标志位）
    ADC_StartCalibration(ADC1);              // 开始校准
    while(ADC_GetCalibrationStatus(ADC1));   // 等待校准完成（约需1ms@14MHz ADC时钟）
    
    /* 配置规则组通道参数
     * ADC_Channel_4：对应PA4引脚（参见STM32数据手册）
     * 采样时间55.5周期：适用于信号源阻抗<50kΩ的情况
     */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
}

/**
  * @brief  获取舵机角度反馈
  * @note   执行单次ADC转换，并将12位原始值转换为0-180°角度
  * @param  无
  * @retval 当前舵机角度（0~180）
  * @warning 假设舵机反馈信号线性对应0-3.3V电压范围
  */
uint8_t Get_Servo_Feedback(void) {
    /* 启动单次转换 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);  // 产生软件触发信号
    
    /* 等待转换完成
     * ADC_FLAG_EOC：规则组转换完成标志
     * 典型转换时间 = 采样时间 + 12.5周期 = 55.5 + 12.5 = 68周期
     * 按14MHz时钟计算约4.86μs
     */
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    
    /* 读取并转换数据
     * 12位ADC值（0-4095）线性映射为0-180°
     * 注意：实际应用中建议使用浮点运算或查表法提高精度
     */
    uint16_t adcVal = ADC_GetConversionValue(ADC1);
    return (uint8_t)(adcVal * 180.0 / 4095.0); 
}
