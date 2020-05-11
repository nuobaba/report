#include <stdint.h>
#include <stdbool.h> 
#include "platform.h"

#if defined( USE_SX1276_RADIO )

#include "spi.h"
#include "../../radio/sx1276-Hal.h"


//需要按单片机型号修改（syn瞎写注释）

//SX1276-SPI复位引脚设置，即lora模块中的RST引脚
#define RESET_IOPORT          GPIOG
#define RESET_PIN             GPIO_Pin_8

//SX1276-SPI片选引脚设置，即lora模块中的SEL引脚
#define NSS_IOPORT            GPIOG
#define NSS_PIN               GPIO_Pin_7    

//SX1276 DIOx引脚定义，这5个引脚需要设置，可以映射到不同的中断
#define DIO0_IOPORT           GPIOG
#define DIO0_PIN              GPIO_Pin_10

#define DIO1_IOPORT           GPIOD
#define DIO1_PIN              GPIO_Pin_10

#define DIO2_IOPORT           GPIOD
#define DIO2_PIN              GPIO_Pin_11

#define DIO3_IOPORT           GPIOD
#define DIO3_PIN              GPIO_Pin_12

#define DIO4_IOPORT           GPIOD
#define DIO4_PIN              GPIO_Pin_13

#define DIO5_IOPORT           GPIOD
#define DIO5_PIN              GPIO_Pin_14

#define RXTX_IOPORT                                 
#define RXTX_PIN              FEM_CTX_PIN

//暂时不知道这一坨干嘛的
#define RXE_PORT       		  GPIOA
#define RXE_PIN  			  GPIO_Pin_2
#define RXE_CLOCK  			  RCC_APB2Periph_GPIOA
#define RXE_HIGH()         	  GPIO_SetBits(RXE_PORT,RXE_PIN)
#define RXE_LOW()          	  GPIO_ResetBits(RXE_PORT,RXE_PIN)
#define RXE_STATE()        	  GPIO_ReadOutputDataBit(RXE_PORT,RXE_PIN)

#define TXE_PORT       		  GPIOA
#define TXE_PIN  			  GPIO_Pin_3
#define TXE_CLOCK  			  RCC_APB2Periph_GPIOA
#define TXE_HIGH()         	  GPIO_SetBits(TXE_PORT,TXE_PIN)
#define TXE_LOW()          	  GPIO_ResetBits(TXE_PORT,TXE_PIN)
#define TXE_STATE()        	  GPIO_ReadOutputDataBit(TXE_PORT,TXE_PIN)

//单片机将射频开关芯片切换成发射状态
void Set_RF_Switch_RX(void)    
{
	RXE_HIGH();
	TXE_LOW();
}

//单片机将射频开关芯片切换成接收状态
void Set_RF_Switch_TX(void)     
{
	RXE_LOW();
	TXE_HIGH();
}

//SX1276引脚初始化
void SX1276_Init_IO( void )     
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOG, ENABLE );

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_InitStructure.GPIO_Pin = NSS_PIN;                     //lora模块的SEL引脚，即片选引脚
    GPIO_Init( NSS_IOPORT, &GPIO_InitStructure );
    GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;           //浮空输入
    GPIO_InitStructure.GPIO_Pin =  DIO0_PIN;
    GPIO_Init( DIO0_IOPORT, &GPIO_InitStructure );
	//GPIO_ResetBits(DIO0_IOPORT,DIO0_PIN);
	
    SPI2_Init();    		                                   //SPI初始化，用的SPI2
}

//设置SX1276的置位和复位
void SX1276SetReset( uint8_t state )                                  
{
    GPIO_InitTypeDef GPIO_InitStructure;
    if(state == RADIO_RESET_ON )
    {
		GPIO_InitStructure.GPIO_Pin = RESET_PIN;              //lora模块的RST引脚，即复位引脚
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
		GPIO_Init( RESET_IOPORT, &GPIO_InitStructure );    
		GPIO_WriteBit( RESET_IOPORT, RESET_PIN, Bit_RESET );  //将该引脚设置为低电平
    }
    else
    {
	    GPIO_InitStructure.GPIO_Pin =  RESET_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init( RESET_IOPORT, &GPIO_InitStructure );	
		GPIO_WriteBit( RESET_IOPORT, RESET_PIN, Bit_SET );    //将该引脚设置为高电平
    }
}

//往寄存器的地址中连续写入几个字节的数据，此函数通常也用来配置寄存器，
//此函数用来与下面的函数配合，当SX1276寄存器是16位的时候，这时候8个字
//节不够用，所以就得用这个函数，往寄存器中写入多个字节。
void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )  
{
    uint8_t i;
	
    GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_RESET );         //片选引脚置0
    SpiInOut( addr | 0x80 );                                 //设置为写
    for( i = 0; i < size; i++ )
    {
        SpiInOut( buffer[i] );                               //开始读寄存器数据，但是一次只能读一个字节的数据
    }
    GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );           //片选引脚置1
}

//往SX1276寄存器地址中里面写入一个字节的数据,此函数通常用来配置寄存器，譬如给某个寄存器某个位写0或者写1。
void SX1276Write( uint8_t addr, uint8_t data )                         
{
    SX1276WriteBuffer( addr, &data, 1 );
}

//将数据写入SX1276的fifo中，此函数用将你要发送的数据写入lora模块中，这样lora才能把你的数据发送出去。
void SX1276WriteFifo( uint8_t *buffer, uint8_t size )                  
{
    SX1276WriteBuffer( 0, buffer, size );
}

//下面2个函数与上面2个读寄存器函数同理，不过它是用来读寄存器值
void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )   
{
    uint8_t i;

    GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_RESET );         //片选引脚置0
    SpiInOut( addr & 0x7F );                                 //设置为读       
    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( 0 );
    }
    GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );           //片选引脚置1
}

void SX1276Read( uint8_t addr, uint8_t *data )  
{
    SX1276ReadBuffer( addr, data, 1 );
}

//在SX1276的fifo中读取数据，当对方有发送数据过来，并且lora成功接收过后，你就需要用此函数去读SX1276fifo中的数据
void SX1276ReadFifo( uint8_t *buffer, uint8_t size )  
{
    SX1276ReadBuffer( 0, buffer, size );
}

/*下面6个函数就是用来读取lora模块那6个引脚的状态，不过在此c语言函数的头文件中把这6个函数定义为了宏定义*/
inline uint8_t SX1276ReadDio0( void )
{
    return GPIO_ReadInputDataBit( DIO0_IOPORT, DIO0_PIN );
}

inline uint8_t SX1276ReadDio1( void )
{
    return GPIO_ReadInputDataBit( DIO1_IOPORT, DIO1_PIN );
}

inline uint8_t SX1276ReadDio2( void )
{
    return GPIO_ReadInputDataBit( DIO2_IOPORT, DIO2_PIN );
}

inline uint8_t SX1276ReadDio3( void )
{
    return GPIO_ReadInputDataBit( DIO3_IOPORT, DIO3_PIN );
}

inline uint8_t SX1276ReadDio4( void )
{
    return GPIO_ReadInputDataBit( DIO4_IOPORT, DIO4_PIN );
}

inline uint8_t SX1276ReadDio5( void )
{
    return GPIO_ReadInputDataBit( DIO5_IOPORT, DIO5_PIN );
}

//射频芯片收发切换，此函数在头文件中被宏定义
inline void SX1276WriteRxTx( uint8_t txEnable )  
{
    if( txEnable != 0 )
    {
		Set_RF_Switch_TX();           //单片机将射频开关芯片切换成发射状态
    }
    else
    {
		Set_RF_Switch_RX();           //单片机将射频开关芯片切换成接收状态
    }
}

#endif 
