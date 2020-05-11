#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "24l01.h" 	 
#include "timer.h"
//SX1278
#include "platform.h"
#include "radio.h"
#include "modbus.h"

#include "sx1276-LoRa.h"
/************************************************
 ALIENTEK精英STM32开发板实验31
 MPU6050六轴传感器 实验     
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

#define SX1278_RX
//#define SX1278_TX

#define BUFFER_SIZE     30                          // Define the payload size here

static uint16_t BufferSize = BUFFER_SIZE;			// RF buffer size
static uint8_t  Buffer[BUFFER_SIZE];				// RF buffer


tRadioDriver *Radio = NULL;
extern tLoRaSettings LoRaSettings;

u8 master=0;//1:主机 2:从机
extern u16 frame_num;//帧标号
u32 rx_timeout_num=0;//接收超时计数
 int main(void)
 {	 
	u8 key,mode;
	u16 i=0,t=0;			 	
	uint8_t txN=0;
	char sendBuf[20];	 
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200
 	LED_Init();		  			//初始化与LED连接的硬件接口
	KEY_Init();					//初始化按键
	LCD_Init();			   		//初始化LCD  
 	POINT_COLOR=RED;			//设置字体为红色 
	LCD_ShowString(30,50,200,16,16,"ELITE STM32");	
	LCD_ShowString(30,70,200,16,16,"NRF24L01 TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2015/1/17"); 
	LCD_ShowString(30,130,200,16,16,"NRF24L01 OK"); 
	printf("------------------硬件连接方式----------------------\r\n");
	printf("板子和sx1278采用spi通信，所以只使用4根线即可，sx1278的中断是通过寄存器查询得到的，所以DIO0-5可以悬空,对了还要加一根复位线\r\n");
	printf("引脚接口:SCL:PB13,MISO:PB14,MOSI:PB15,NSS:PG7,RESET:PG8\r\n");
	printf("我使用的是板子nrf24l01的接口\r\n");
	printf("-----------------------end--------------------------\r\n");
	printf("------------------使用方法--------------------------\r\n");
	printf("本实验用于精英版，lora使用sx1278模块，spi通信\r\n");
	printf("1.点击key0，模块处于循环接收状态，可接收当前信道的值\r\n");
	printf("2.点击key1，模块处于主机模式，一直向目标地址发送数据，从机收到数据之后会响应ack，主机收到ack通过验证之后会发送下一帧数据,如果没有收到ack，则一直重发当前帧\r\n");
  printf("3.点击key up，模块处于从机模式，用于和主机通信\r\n");
	printf("----------------------------------------------------\r\n");
	TIM3_Int_Init(10,7199);//1ms
	Modbus_Init();
	Radio = RadioDriverInit();
	Radio->Init();
	
 	while(1)
	{	
		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{
			mode=0;   
			break;
		}else if(key==KEY1_PRES)
		{
			mode=1;
			break;
		}else if(key==WKUP_PRES)
		{
			mode=2;
			break;
		}
		t++;
		if(t==100)LCD_ShowString(10,150,230,16,16,"KEY0:RX_Mode  KEY1:TX_Mode"); //闪烁显示提示信息
 		if(t==200)
		{	
			LCD_Fill(10,150,230,150+16,WHITE);
			t=0; 
		}
		delay_ms(5);	  
	}   
 	LCD_Fill(10,150,240,166,WHITE);//清空上面的显示		  
 	POINT_COLOR=BLUE;//设置字体为蓝色	   
	if(mode==0)//RX模式
	{
		printf("循环接收模式:\r\n");
		LCD_ShowString(30,150,200,16,16,"NRF24L01 RX_Mode");	
		LCD_ShowString(30,170,200,16,16,"Received DATA:");	
		Radio->StartRx();   //RFLR_STATE_RX_INIT  
		while(1)
		{	  		    		    				 
			while( Radio->Process( ) == RF_RX_DONE)//一旦接收到信息,则显示出来.
			{			
				Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );			
				if( strlen(Buffer) != 0 )
				{
					Buffer[strlen(Buffer)]=0;//加入字符串结束符
					printf("LoRa RX: %s\r\n",Buffer);		
					LCD_ShowString(0,190,lcddev.width-1,32,16,Buffer);   					
					LED1=!LED1;
					for(i=0;i<BUFFER_SIZE;i++)
						Buffer[i] = 0;
				}
				Radio->StartRx( );			 
			} 				    
		}
	}else if(mode==1)
	{		
		printf("主机模式:\r\n");
		master=1;
		LoRaSettings.RxSingleOn = true;//单次接收状态
		LCD_ShowString(30,150,200,16,16,"NRF24L01 TX_Mode");
		Modbus_PING();
		while(1)
		{
			switch(Radio->Process())
			{
				case RF_RX_TIMEOUT:
					printf("RF_RX_TIMEOUT...%d\r\n",rx_timeout_num);
					rx_timeout_num++;
					Modbus_PING();
					break;
				case RF_RX_DONE:
					Radio->GetRxPacket(Buffer, ( uint16_t* )&BufferSize);
				  if(BufferSize!=0)
					{
						//打印数据
						printf("Rx:");
						for(i=0;i<BufferSize;i++)
						{
							printf("%2x ",Buffer[i]);
						}
						printf("\r\n");
						if(Modbus_Service(Buffer,BufferSize)==0)
						{
							printf("第%d条通信,响应成功\r\n",frame_num);
							frame_num++;//通信成功+1
							Modbus_PING();//继续通信
						}						
					}
					break;
				case RF_TX_DONE:
					printf("RF_TX_DONE\r\n");
					Radio->StartRx( );
					break;
			}
			//delay_ms(100);		
		}
	}	
	else if(mode == 2)
	{
		printf("从机模式:\r\n");
		master=2;
		LoRaSettings.RxSingleOn = false;//循环接收状态
		LCD_ShowString(30,150,200,16,16,"NRF24L01 TX_RX_Mode");
		Radio->StartRx( );
		while(1)
		{
			switch(Radio->Process())
			{
				case RF_RX_TIMEOUT:
					printf("RF_RX_TIMEOUT\r\n");				
					break;
				case RF_RX_DONE:
					Radio->GetRxPacket(Buffer, ( uint16_t* )&BufferSize);
				  if(BufferSize!=0)
					{
						//打印数据
						printf("Rx:");
						for(i=0;i<BufferSize;i++)
						{
							printf("%2x ",Buffer[i]);
						}
						printf("\r\n");
						Modbus_Service(Buffer,BufferSize);
					}
					break;
				case RF_TX_DONE:
					printf("RF_TX_DONE\r\n");
					Radio->StartRx( );
					break;
			}
			//delay_ms(100);		
		}
	}
}


