#include "stm32f10x.h"
#include "system.h"

extern volatile uint32_t timer_counter;
int main(void){
//		SystemInit();
//		pinlv = SystemCoreClock;
		Hardware_Init();
		ADC_Battery_Init();      // 电池电压检测初始化（PA4 = ADC_IN4）
		OLED_ShowString(1, 1, "HelloWorld!");
		ShowResetReason();        // 显示复位原因（第2/3行）
		Self_Check_Routine();
		Lift_FindHome();          // 上电找零：升降往下走到碰底端限位开关
		WDG_Init();               // 自检完成后启动独立看门狗
 while(1){
	// OLED_ShowNum(2,1,timer_counter,8);
	System_StateMachine();
	WDG_Feed();                  // 每轮主循环喂狗
 }
 }
