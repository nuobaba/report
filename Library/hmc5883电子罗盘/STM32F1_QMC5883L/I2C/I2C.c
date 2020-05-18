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

//����IIC��ʼ�ź�
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

//����IICֹͣ�ź�
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

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0; 
	IIC_SDA=1;delay_us(5);
	SDA_IN();      //SDA����Ϊ����	   
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
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
}


void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA=1;
		else
			IIC_SDA=0;
		txd<<=1; 	  
		delay_us(5);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delay_us(5); 
		IIC_SCL=0;	
		delay_us(5);
    }	 
} 

u8 IIC_Read_Byte(void )
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
