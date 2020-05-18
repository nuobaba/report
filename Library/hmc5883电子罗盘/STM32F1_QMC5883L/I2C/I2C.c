#include "I2C.h"
#include "delay.h"

void i2c_init(void)
{
	GPIO_InitTypeDef GPIO_Init_Structure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_Init_Structure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_11;
	GPIO_Init_Structure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init_Structure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Init_Structure);
	GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_11);
}

void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}
void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();   
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(5);
 	IIC_SDA=0;
	delay_us(5);
	IIC_SCL=0;
}

//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();
	//IIC_SCL=0;
	IIC_SDA=0;
	IIC_SCL=1;
 	delay_us(5);
	//IIC_SCL=1; 
	IIC_SDA=1;
	delay_us(5);							   	
}

void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(5);
	IIC_SCL=1;
	delay_us(5);
	IIC_SCL=0;
}

void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(5);
	IIC_SCL=1;
	delay_us(5);
	IIC_SCL=0;
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0; 
	IIC_SDA=1;delay_us(5);
	SDA_IN();      //SDA设置为输入	   
	IIC_SCL=1;delay_us(5); 	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			//IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
}


void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA=1;
		else
			IIC_SDA=0;
		txd<<=1; 	  
		delay_us(5);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(5); 
		IIC_SCL=0;	
		delay_us(5);
    }	 
} 

u8 IIC_Read_Byte(void )
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(5);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(5); 
    }
	return receive;
}
