#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
 				  	    													  
void SPI2_Init(void);			 //��ʼ��SPI��
void SPI2_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI���߶�дһ���ֽ�
uint8_t SpiInOut( uint8_t outData );
#endif

