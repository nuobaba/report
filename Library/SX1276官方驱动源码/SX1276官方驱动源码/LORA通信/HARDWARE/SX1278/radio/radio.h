#ifndef __RADIO_H__
#define __RADIO_H__

#include "sys.h"

//SX1272和SX1276通用参数定义
#define LORA                                        1         // [0: OFF, 1: ON]

//RF过程函数返回码
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

//定义不同函数指针的单选驱动程序结构
typedef struct sRadioDriver
{
    void ( *Init )( void );                                     //SX1276初始化
    void ( *Reset )( void );                                    //SX1276复位 
    void ( *StartRx )( void );                                  //SX1276开始接收
    void ( *GetRxPacket )( void *buffer, uint16_t *size );      //SX1276接受数据
    void ( *SetTxPacket )( const void *buffer, uint16_t size ); //SX1276发送数据
    uint32_t ( *Process )( void );
}tRadioDriver;

tRadioDriver* RadioDriverInit( void );                          //用来初始化上面的几个函数

#endif 
