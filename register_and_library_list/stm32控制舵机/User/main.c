#include "stm32f10x.h"
#include "bsp_Advance_tim.h"
#include "delay.h"

int main(void)
{
	int delay_time;
	delay_init();
	TIM_Init();
	
	delay_time = 500;
	while(1)
	{
		delay_ms(delay_time);
		TIM_SetCompare1(ADVANCE_TIM, 175);
		delay_ms(delay_time);
		TIM_SetCompare1(ADVANCE_TIM, 180);
		delay_ms(delay_time);
		TIM_SetCompare1(ADVANCE_TIM, 185);
		delay_ms(delay_time);
		TIM_SetCompare1(ADVANCE_TIM, 190);
		delay_ms(delay_time);
		TIM_SetCompare1(ADVANCE_TIM, 170);
	}
}


