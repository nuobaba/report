#include <stdint.h> 
#include "spi.h"
#include "sx12xxEiger.h"

volatile uint32_t TickCounter = 0;    //此变量来记录时间，在定时3中断引用，每1ms加一次

//时钟一类外设的初始化定义
void BoardInit( void )
{
    //为1us中断设置SysTick定时器(不太频繁以节省电源)
    if( SysTick_Config( SystemCoreClock / 1000 ) )    //1ms
    { 
        while (1);
    }
    // Initialize SPI
    //SpiInit( );
}

//延时函数，delay是多少就延时多少ms
void Delay ( uint32_t delay )                 
{  
    uint32_t startTick = TickCounter;
    while( ( TickCounter - startTick ) < delay );   
}

//延时函数，delay是多少就延时多少s
void LongDelay ( uint8_t delay ) 
{
    uint32_t longDelay;
    uint32_t startTick;
    longDelay = delay * 1000;
    startTick = TickCounter;
    while( ( TickCounter - startTick ) < longDelay );   
}
