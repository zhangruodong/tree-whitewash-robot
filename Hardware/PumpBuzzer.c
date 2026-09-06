#include "stm32f10x.h"                  // Device header

void BUMP_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // 开启GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 配置PA0为
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 可选：初始化时拉低（根据硬件设计调整）
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
}

void BUMP_KAI(void){
GPIO_SetBits(GPIOA, GPIO_Pin_0);
}

void BUMP_GUAN(void){
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);
}

/*------------------------------------------蜂鸣器---------------------------------------------*/
void FMQ_Init(void) {
    	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//开启GPIOB的时钟
															//使用各个外设前必须开启时钟，否则对外设的操作无效
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;					//定义结构体变量
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//GPIO模式，赋值为推挽输出模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;				//GPIO引脚，赋值为第12号引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//GPIO速度，赋值为50MHz
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//将赋值后的构体变量传递给GPIO_Init函数
															//函数内部会自动根据结构体的参数配置相应寄存器
															//实现GPIOB的
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	
}
void  FMQ_KAI(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
}

void  FMQ_GUAN(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
}
