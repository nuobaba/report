#ifndef __SX1276_HAL_H__
#define __SX1276_HAL_H__
#include "stdint.h"

//�궨�巨�����Ӽ��Ĵ��룬��Щ���������ͷ�ļ��¶�������
#define DIO0                                        SX1276ReadDio0( )
#define DIO1                                        SX1276ReadDio1( )
#define DIO2                                        SX1276ReadDio2( )
#define DIO3                                        SX1276ReadDio3( )
#define DIO4                                        SX1276ReadDio4( )
#define DIO5                                        SX1276ReadDio5( )

//��ƵоƬ�շ��л���1��TX��0��RX�����ú궨����Ӽ���˴��묩
#define RXTX( txEnable )                            SX1276WriteRxTx( txEnable ); 

//���Եõ���ʱTickCounterֵ
#define GET_TICK_COUNT( )                           ( TickCounter )  //�˱���ÿ1ms����һ�Σ��ڶ�ʱ��3�ж�������
#define TICK_RATE_MS( ms )                          ( ms )           //�˺������൱�ڶ���msΪ����

typedef enum
{
    RADIO_RESET_OFF,                             
    RADIO_RESET_ON,                              
}tRadioResetState;


void SX1276_Init_IO( void );                      //��ʼ��IO�ӿ�

void SX1276SetReset( uint8_t state );             //����loraģ�����λ�͸�λ

void SX1276Write( uint8_t addr, uint8_t data );   //��ָ���Ĵ�����ַ������д������

void SX1276Read( uint8_t addr, uint8_t *data );   //��ȡ�Ĵ���ָ����ַ������

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );  //���Ĵ����ĵ�ַ������д�뼸������

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );   //�ڼĴ����ĵ�ַ��������ȡ��������

void SX1276WriteFifo( uint8_t *buffer, uint8_t size );     //������д��SX1276��fifo��

void SX1276ReadFifo( uint8_t *buffer, uint8_t size );      //��SX1276��fifo�ж�ȡ����


inline uint8_t SX1276ReadDio0( void );  //��ȡSX1276DIO0���ŵ�״̬

inline uint8_t SX1276ReadDio1( void );	//��ȡSX1276DIO1���ŵ�״̬

inline uint8_t SX1276ReadDio2( void );	//��ȡSX1276DIO2���ŵ�״̬

inline uint8_t SX1276ReadDio3( void );	//��ȡSX1276DIO3���ŵ�״̬

inline uint8_t SX1276ReadDio4( void );	//��ȡSX1276DIO4���ŵ�״̬

inline uint8_t SX1276ReadDio5( void );	//��ȡSX1276DIO5���ŵ�״̬


inline void SX1276WriteRxTx( uint8_t txEnable );  //��ƵоƬ�շ��л���1��TX��0��RX��

void Set_RF_Switch_RX(void);            //��Ƭ������Ƶ����оƬ�л��ɷ���״̬

void Set_RF_Switch_TX(void);            //��Ƭ������Ƶ����оƬ�л��ɽ���״̬

#endif 
