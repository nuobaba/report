#ifndef __SX1276_HAL_H__
#define __SX1276_HAL_H__
#include "stdint.h"

//宏定义法，更加简洁的代码，这些代码在这个头文件下都定义了
#define DIO0                                        SX1276ReadDio0( )
#define DIO1                                        SX1276ReadDio1( )
#define DIO2                                        SX1276ReadDio2( )
#define DIO3                                        SX1276ReadDio3( )
#define DIO4                                        SX1276ReadDio4( )
#define DIO5                                        SX1276ReadDio5( )

//射频芯片收发切换（1：TX，0：RX），用宏定义更加简洁了代码
#define RXTX( txEnable )                            SX1276WriteRxTx( txEnable ); 

//可以得到此时TickCounter值
#define GET_TICK_COUNT( )                           ( TickCounter )  //此变量每1ms都加一次，在定时器3中断中引用
#define TICK_RATE_MS( ms )                          ( ms )           //此函数就相当于定义ms为多少

typedef enum
{
    RADIO_RESET_OFF,                             
    RADIO_RESET_ON,                              
}tRadioResetState;


void SX1276_Init_IO( void );                      //初始化IO接口

void SX1276SetReset( uint8_t state );             //设置lora模块的置位和复位

void SX1276Write( uint8_t addr, uint8_t data );   //往指定寄存器地址中里面写入数据

void SX1276Read( uint8_t addr, uint8_t *data );   //读取寄存器指定地址的数据

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );  //往寄存器的地址中连续写入几个数据

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );   //在寄存器的地址中连续读取几个数据

void SX1276WriteFifo( uint8_t *buffer, uint8_t size );     //将数据写入SX1276的fifo中

void SX1276ReadFifo( uint8_t *buffer, uint8_t size );      //在SX1276的fifo中读取数据


inline uint8_t SX1276ReadDio0( void );  //获取SX1276DIO0引脚的状态

inline uint8_t SX1276ReadDio1( void );	//获取SX1276DIO1引脚的状态

inline uint8_t SX1276ReadDio2( void );	//获取SX1276DIO2引脚的状态

inline uint8_t SX1276ReadDio3( void );	//获取SX1276DIO3引脚的状态

inline uint8_t SX1276ReadDio4( void );	//获取SX1276DIO4引脚的状态

inline uint8_t SX1276ReadDio5( void );	//获取SX1276DIO5引脚的状态


inline void SX1276WriteRxTx( uint8_t txEnable );  //射频芯片收发切换（1：TX，0：RX）

void Set_RF_Switch_RX(void);            //单片机将射频开关芯片切换成发射状态

void Set_RF_Switch_TX(void);            //单片机将射频开关芯片切换成接收状态

#endif 
