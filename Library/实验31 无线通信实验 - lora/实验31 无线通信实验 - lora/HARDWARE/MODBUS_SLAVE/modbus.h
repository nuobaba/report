#ifndef _MODBUS_H
#define _MODBUS_H
#include "sys.h"
#include "radio.h"

#define MODBUS_ID_LEN 5//modbus协议中的id长度

#define MODBUS_FUN_ADD MODBUS_ID_LEN//功能码的位置
#define MODBUS_STREG_ADD MODBUS_FUN_ADD+1//寄存器起始地址
#define MODBUS_LEN_ADD MODBUS_STREG_ADD+1//寄存器长度
#define MODBUS_NUM_ADD MODBUS_LEN_ADD+1//modbus帧编号
#define MODBUS_DATA_ADD MODBUS_NUM_ADD+1//modbus帧编号

#define MODBUS_MAX_TX 255//最大modbus帧
#define MODBUS_MAX_RX 255//最大modbus帧
#define MODBUS_DATA_MAX_LEN 255-6-MODBUS_ID_LEN//modbus数据帧中数据区的最大长度


typedef struct _Frame{
  uint8_t id[MODBUS_ID_LEN];
  uint8_t functionCode;
  uint8_t regStart;
  uint8_t regLen;
	uint8_t data[4];
  uint8_t crcH;
  uint8_t crcL;
}Frame;
extern Frame* tF;
extern Frame* rF;
//本机id
extern uint8_t UID_data[];
extern tRadioDriver *Radio;

void Modbus_Init(void);
void Modbus_SendData(u8 len);
u8 Modbus_Service(u8 *buf,u8 len);
void Modbus_02_Solve(void);
void Modbus_01_Solve(void);
void Modbus_05_Solve(void);
void Modbus_15_Solve(void);
void Modbus_03_Solve(void);
void Modbus_06_Solve(void);
void Modbus_16_Solve(void);

void Modbus_PING(void);

#endif
