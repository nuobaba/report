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
 ALIENTEK��ӢSTM32������ʵ��31
 MPU6050���ᴫ���� ʵ��     
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

#define SX1278_RX
//#define SX1278_TX

#define BUFFER_SIZE     30                          // Define the payload size here

static uint16_t BufferSize = BUFFER_SIZE;			// RF buffer size
static uint8_t  Buffer[BUFFER_SIZE];				// RF buffer


tRadioDriver *Radio = NULL;
extern tLoRaSettings LoRaSettings;

u8 master=0;//1:���� 2:�ӻ�
extern u16 frame_num;//֡���
u32 rx_timeout_num=0;//���ճ�ʱ����
 int main(void)
 {	 
	u8 key,mode;
	u16 i=0,t=0;			 	
	uint8_t txN=0;
	char sendBuf[20];	 
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
 	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();					//��ʼ������
	LCD_Init();			   		//��ʼ��LCD  
 	POINT_COLOR=RED;			//��������Ϊ��ɫ 
	LCD_ShowString(30,50,200,16,16,"ELITE STM32");	
	LCD_ShowString(30,70,200,16,16,"NRF24L01 TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2015/1/17"); 
	LCD_ShowString(30,130,200,16,16,"NRF24L01 OK"); 
	printf("------------------Ӳ�����ӷ�ʽ----------------------\r\n");
	printf("���Ӻ�sx1278����spiͨ�ţ�����ֻʹ��4���߼��ɣ�sx1278���ж���ͨ���Ĵ�����ѯ�õ��ģ�����DIO0-5��������,���˻�Ҫ��һ����λ��\r\n");
	printf("���Žӿ�:SCL:PB13,MISO:PB14,MOSI:PB15,NSS:PG7,RESET:PG8\r\n");
	printf("��ʹ�õ��ǰ���nrf24l01�Ľӿ�\r\n");
	printf("-----------------------end--------------------------\r\n");
	printf("------------------ʹ�÷���--------------------------\r\n");
	printf("��ʵ�����ھ�Ӣ�棬loraʹ��sx1278ģ�飬spiͨ��\r\n");
	printf("1.���key0��ģ�鴦��ѭ������״̬���ɽ��յ�ǰ�ŵ���ֵ\r\n");
	printf("2.���key1��ģ�鴦������ģʽ��һֱ��Ŀ���ַ�������ݣ��ӻ��յ�����֮�����Ӧack�������յ�ackͨ����֤֮��ᷢ����һ֡����,���û���յ�ack����һֱ�ط���ǰ֡\r\n");
  printf("3.���key up��ģ�鴦�ڴӻ�ģʽ�����ں�����ͨ��\r\n");
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
		if(t==100)LCD_ShowString(10,150,230,16,16,"KEY0:RX_Mode  KEY1:TX_Mode"); //��˸��ʾ��ʾ��Ϣ
 		if(t==200)
		{	
			LCD_Fill(10,150,230,150+16,WHITE);
			t=0; 
		}
		delay_ms(5);	  
	}   
 	LCD_Fill(10,150,240,166,WHITE);//����������ʾ		  
 	POINT_COLOR=BLUE;//��������Ϊ��ɫ	   
	if(mode==0)//RXģʽ
	{
		printf("ѭ������ģʽ:\r\n");
		LCD_ShowString(30,150,200,16,16,"NRF24L01 RX_Mode");	
		LCD_ShowString(30,170,200,16,16,"Received DATA:");	
		Radio->StartRx();   //RFLR_STATE_RX_INIT  
		while(1)
		{	  		    		    				 
			while( Radio->Process( ) == RF_RX_DONE)//һ�����յ���Ϣ,����ʾ����.
			{			
				Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );			
				if( strlen(Buffer) != 0 )
				{
					Buffer[strlen(Buffer)]=0;//�����ַ���������
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
		printf("����ģʽ:\r\n");
		master=1;
		LoRaSettings.RxSingleOn = true;//���ν���״̬
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
						//��ӡ����
						printf("Rx:");
						for(i=0;i<BufferSize;i++)
						{
							printf("%2x ",Buffer[i]);
						}
						printf("\r\n");
						if(Modbus_Service(Buffer,BufferSize)==0)
						{
							printf("��%d��ͨ��,��Ӧ�ɹ�\r\n",frame_num);
							frame_num++;//ͨ�ųɹ�+1
							Modbus_PING();//����ͨ��
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
		printf("�ӻ�ģʽ:\r\n");
		master=2;
		LoRaSettings.RxSingleOn = false;//ѭ������״̬
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
						//��ӡ����
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


