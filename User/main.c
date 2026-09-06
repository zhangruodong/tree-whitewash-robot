#include "stm32f10x.h"
#include "system.h"

extern volatile uint32_t timer_counter;
int main(void){
//		SystemInit();
//		pinlv = SystemCoreClock;
		Hardware_Init();
		OLED_ShowString(1, 1, "HelloWorld!");
		Self_Check_Routine();
	 while(1){		 
		OLED_ShowNum(2,1,timer_counter,8);
		System_StateMachine();	
		
		 
	 }
 }
