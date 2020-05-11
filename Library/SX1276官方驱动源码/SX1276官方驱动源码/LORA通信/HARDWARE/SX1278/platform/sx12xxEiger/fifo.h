#ifndef __FIFO_H__
#define __FIFO_H__

#include <stdbool.h>

#if defined( STM32F4XX )
    #include "stm32f4xx.h"
#elif defined( STM32F2XX )
    #include "stm32f2xx.h"
#else
    #include "stm32f10x.h"
#endif

typedef struct sFifo      //fifo结构体
{
	uint16_t Begin;
	uint16_t End;
	uint16_t *Data;
    uint16_t Size;
}tFifo;

void FifoInit( tFifo *fifo, uint16_t *buffer, uint16_t size );   //fifo初始化

void FifoPush( tFifo *fifo, uint16_t data );                     //把数据写入fifo中        

uint16_t FifoPop( tFifo *fifo );                                 //在fifo中读数据  

void FifoFlush( tFifo *fifo );                                   //将fifo的起始位置和结束位置都设置为0

bool IsFifoEmpty( tFifo *fifo );                                 //判断fifo是否为空

bool IsFifoFull( tFifo *fifo );                                  //判断fifo是否满了

#endif 
