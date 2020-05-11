#ifndef __RADIO_H__
#define __RADIO_H__

#include "sys.h"

//SX1272��SX1276ͨ�ò�������
#define LORA                                        1         // [0: OFF, 1: ON]

//RF���̺���������
typedef enum
{
    RF_IDLE,
    RF_BUSY,
    RF_RX_DONE,
    RF_RX_TIMEOUT,
    RF_TX_DONE,
    RF_TX_TIMEOUT,
    RF_LEN_ERROR,
    RF_CHANNEL_EMPTY,
    RF_CHANNEL_ACTIVITY_DETECTED,
}tRFProcessReturnCodes;

//���岻ͬ����ָ��ĵ�ѡ��������ṹ
typedef struct sRadioDriver
{
    void ( *Init )( void );                                     //SX1276��ʼ��
    void ( *Reset )( void );                                    //SX1276��λ 
    void ( *StartRx )( void );                                  //SX1276��ʼ����
    void ( *GetRxPacket )( void *buffer, uint16_t *size );      //SX1276��������
    void ( *SetTxPacket )( const void *buffer, uint16_t size ); //SX1276��������
    uint32_t ( *Process )( void );
}tRadioDriver;

tRadioDriver* RadioDriverInit( void );                          //������ʼ������ļ�������

#endif 
