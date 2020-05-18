#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "stm32f10x.h"


#define WRITE_ADDRESS   0x1a
#define READ_ADDRESS    0x1b

#define CONFIGA         0x09
#define CONFIGB         0x09
#define MODE            0x09

#define DATAX_L         0x00
#define DATAX_M         0x01
#define DATAY_L         0x02
#define DATAY_M         0x03
#define DATAZ_L         0x04
#define DATAZ_M         0x05

void hmc_write_reg(u8 reg,u8 data);
u8 hmc_read_reg(u8 reg);
void hmc_read_XYZ(short int *data);
void hmc_init(void);

#endif

