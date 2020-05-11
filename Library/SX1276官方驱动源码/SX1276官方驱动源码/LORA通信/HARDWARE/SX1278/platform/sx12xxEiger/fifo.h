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

typedef struct sFifo      //fifo�ṹ��
{
	uint16_t Begin;
	uint16_t End;
	uint16_t *Data;
    uint16_t Size;
}tFifo;

void FifoInit( tFifo *fifo, uint16_t *buffer, uint16_t size );   //fifo��ʼ��

void FifoPush( tFifo *fifo, uint16_t data );                     //������д��fifo��        

uint16_t FifoPop( tFifo *fifo );                                 //��fifo�ж�����  

void FifoFlush( tFifo *fifo );                                   //��fifo����ʼλ�úͽ���λ�ö�����Ϊ0

bool IsFifoEmpty( tFifo *fifo );                                 //�ж�fifo�Ƿ�Ϊ��

bool IsFifoFull( tFifo *fifo );                                  //�ж�fifo�Ƿ�����

#endif 
