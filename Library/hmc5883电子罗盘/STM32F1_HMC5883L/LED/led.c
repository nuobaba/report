 #include "led.h"
 //PC0,1,2,3口输出高电平
 //LED初始化
 void led_init(void)
 {
 	   GPIO_InitTypeDef GPIO_Init_Structure;
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	   GPIO_Init_Structure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	   GPIO_Init_Structure.GPIO_Mode=GPIO_Mode_Out_PP;
	   GPIO_Init_Structure.GPIO_Speed=GPIO_Speed_50MHz;
	   GPIO_Init(GPIOC,&GPIO_Init_Structure);
	   GPIO_SetBits(GPIOC,GPIO_Pin_0);
	   GPIO_SetBits(GPIOC,GPIO_Pin_1);
	   GPIO_SetBits(GPIOC,GPIO_Pin_2);
	   GPIO_SetBits(GPIOC,GPIO_Pin_3);
 }
