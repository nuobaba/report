#include <stdint.h> 
#include "spi.h"
#include "sx12xxEiger.h"

volatile uint32_t TickCounter = 0;    //�˱�������¼ʱ�䣬�ڶ�ʱ3�ж����ã�ÿ1ms��һ��

//ʱ��һ������ĳ�ʼ������
void BoardInit( void )
{
    //Ϊ1us�ж�����SysTick��ʱ��(��̫Ƶ���Խ�ʡ��Դ)
    if( SysTick_Config( SystemCoreClock / 1000 ) )    //1ms
    { 
        while (1);
    }
    // Initialize SPI
    //SpiInit( );
}

//��ʱ������delay�Ƕ��پ���ʱ����ms
void Delay ( uint32_t delay )                 
{  
    uint32_t startTick = TickCounter;
    while( ( TickCounter - startTick ) < delay );   
}

//��ʱ������delay�Ƕ��پ���ʱ����s
void LongDelay ( uint8_t delay ) 
{
    uint32_t longDelay;
    uint32_t startTick;
    longDelay = delay * 1000;
    startTick = TickCounter;
    while( ( TickCounter - startTick ) < longDelay );   
}
