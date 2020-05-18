#include "stm32f10x.h"
#include "I2C.h"
#include "HMC5883L.h"
#include "delay.h"
#include "math.h"
#include "usart.h"

//Á¬½ÓÒý½ÅPA8(SCL)PA11(SDA)

short int data[3];
double angle;
int main(void)
{	
	delay_init();
	i2c_init();
	hmc_init();
	uart_init(115200);
	while(1)
	{
		hmc_read_XYZ(data);
		angle=atan2((double)data[0],(double)data[2])*(180 / 3.14159265)+180;
		printf("%f   \r\n",angle);
		delay_ms(200);
	}
}

