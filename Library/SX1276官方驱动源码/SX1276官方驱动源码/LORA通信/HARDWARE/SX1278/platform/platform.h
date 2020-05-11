#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#ifndef __GNUC__
#define inline
#endif

//ƽ̨�Ķ���
#define Bleeper                                     3
#define SX1243ska                                   2
#define SX12xxEiger                                 1
#define SX12000DVK                                  0

//ƽ̨��ѡ��
//��ȡ��ƽ̨���岢ѡ������ƽ̨�����ڱ���������ѡ�������/����ƽ̨����
#define PLATFORM                                    SX12xxEiger

#if( PLATFORM == SX12xxEiger )     //���PLATFORM == SX12xxEiger�Ͷ�������Ķ���

	//loraģ���ͺŵ�ѡ��
	//��ȡ���������̨��ע�ͣ���������ƽ̨����ע�ͣ����ڱ���������ѡ�������/���������ͺŶ���
	#define USE_SX1276_RADIO
	//#define USE_SX1232_RADIO
	//#define USE_SX1272_RADIO
	//#define USE_SX1243_RADIO

	//ģ���ѡ��
	//SX1276��������ģ�顣�뽫���ӵ�ģ������Ϊֵ1����������ģ������Ϊ0
	#ifdef USE_SX1276_RADIO
		#define MODULE_SX1276RF1IAS                     0
		#define MODULE_SX1276RF1JAS                     0
		#define MODULE_SX1276RF1KAS                     1
	#endif

	#include "sx12xxEiger/sx12xxEiger.h"
	#define USE_UART                                     0

#elif( PLATFORM == SX12000DVK )   //���PLATFORM == SX12000DVK�Ͷ�������Ķ���

	#define USE_SX1276_RADIO
	//#define USE_SX1232_RADIO
	//#define USE_SX1272_RADIO
	//#define USE_SX1243_RADIO
	#include "sx1200dvk/sx1200dvk.h"

#elif( PLATFORM == SX1243ska )   //���PLATFORM == SX1243ska

#elif( PLATFORM == Bleeper )     //���PLATFORM == Bleeper�Ͷ�������Ķ���
    #define USE_SX1272_RADIO
    
    #include "bleeper/bleeper.h"
    #define USE_UART                                       0

#else                         //���PLATFORMû�ж���
    #error "Missing define: Platform (ie. SX12xxEiger)"
#endif


void Soft_delay_us(u16 time);
void Soft_delay_ms(u16 time);


#endif 
