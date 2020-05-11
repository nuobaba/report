#ifndef __SX1276_H__
#define __SX1276_H__

#include <stdint.h>
#include <stdbool.h>
#include "sys.h"
extern uint8_t SX1276Regs[0x70];     						 //SX1276�Ĵ�������

void SX1276_Init( void );									 //��ʼ��SX1276

void SX1276_Reset( void );									 //����SX1276

/*���º�����û�б�ʹ�õ�����Ϊ��sx1276-LoRa.h�����ֶ�����һϵ��������������ͬ�ĺ���*/
void SX1276_SetLoRaOn( bool enable ); 						 //����LoRa���ƽ������FSK���ƽ����

bool SX1276_GetLoRaOn( void );								 //��ȡLoRa���ƽ����״̬

void SX1276SetOpMode( uint8_t opMode );  					 //����SX1276����ģʽ

uint8_t SX1276_GetOpMode( void );     						 //��ȡSX1276����ģʽ

uint8_t SX1276_ReadRxGain( void );   						 //��ȡ��ǰRx��������

double SX1276ReadRssi( void );       						 //��ȡ�����ź�ǿ��

uint8_t SX1276_GetPacketRxGain( void );                      //��ȡ����ʱ������ֵ

int8_t SX1276_GetPacketSnr( void );							 //��ȡ����ʱ�������ֵ���źź������ı�ֵ�������Խ�ߣ�˵���źŸ���ԽС��

double SX1276_GetPacketRssi( void );  					     //��ȡ�����ǵ������ź�ǿ��

/*!
 * \brief Gets the AFC value measured while receiving the packet
 *
 * \retval afcValue Current AFC value in [Hz]
 */
uint32_t SX1276GetPacketAfc( void );                          //�˺�����֪������


void SX1276StartRx( void );                                   //��ʼ����

void SX1276GetRxPacket( void *buffer, uint16_t *size );       //�õ����յ�����

void SX1276SetTxPacket( const void *buffer, uint16_t size );  //��������

uint8_t SX1276GetRFState( void );       				      //�õ�RFLRState״̬

void SX1276SetRFState( uint8_t state );						  //����RFLRState״̬��RFLRState��ֵ����������ĺ���������һ���Ĵ���

uint32_t SX1276Process( void );         			          //SX1276ģ��ӷ������ݵĴ�����

//���ӵĺ���
/*************************************************************/
//u8 		SX1276Dio0State(void);
void 	SX1276RxStateEnter( void );
void 	SX1276RxDataRead(uint8_t *pbuf, uint8_t *size );
u8 		SX1276TxData(uint8_t *pbuf, uint8_t size );
void 	SX1276TxPower(uint8_t pwr );
void 	SX1276FreqSet(uint32_t freq);
u8 		SX1276CheckSPI(void);
u8 		SX1276TxDataCRC32(uint8_t *pbuf, uint8_t size );
u8 		SX1276RxDataReadCRC32(uint8_t *pbuf, uint8_t *size );
/**************************************************************/
#endif 
