#ifndef __LED_H
#define __LED_H

#include "sys.h"
//LED端口定义
#define LED0 PCout(0)
#define LED1 PCout(1)
#define LED2 PCout(2)
#define LED3 PCout(3)
void led_init(void);//初始化

#endif
