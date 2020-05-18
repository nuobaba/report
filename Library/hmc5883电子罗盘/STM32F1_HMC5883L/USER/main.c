#include "stm32f10x.h"
#include "I2C.h"
#include "HMC5883L.h"
#include "delay.h"
#include "math.h"
#include "usart.h"
short int data[3];
double angle;
int main(void)
{	
	delay_init();
	i2c_init();
	hmc_init();
	uart_init(9600);
	while(1)
	{
		hmc_read_XYZ(data);
		angle=atan2((double)data[2],(double)data[0])*(180 / 3.14159265)+180;
		delay_ms(200);
	}
}

