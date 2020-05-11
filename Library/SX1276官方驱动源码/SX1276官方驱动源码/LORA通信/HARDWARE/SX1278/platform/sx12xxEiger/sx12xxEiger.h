#ifndef __SX12XXEIGER_H__
#define __SX12XXEIGER_H__

#include <stdint.h>
#include <stdbool.h>

#if defined( STM32F4XX )
    #include "stm32f4xx.h"
#elif defined( STM32F2XX )
    #include "stm32f2xx.h"
#else
    #include "stm32f10x.h"
#endif

#define USE_USB                                     	1

#if defined( STM32F4XX ) || defined( STM32F2XX )
	#define BACKUP_REG_BOOTLOADER                       RTC_BKP_DR0      /* Booloader enter*/
#else
	#define BACKUP_REG_BOOTLOADER                       BKP_DR1          /* Booloader enter*/
#endif

#define FW_VERSION                                  	"2.0.B2"
#define SK_NAME                                     	"SX12xxEiger"

//SX1276的状态枚举
typedef enum
{
    SX_OK,
    SX_ERROR,
    SX_BUSY,
    SX_EMPTY,
    SX_DONE,
    SX_TIMEOUT,
    SX_UNSUPPORTED,
    SX_WAIT,
    SX_CLOSE,
    SX_YES,
    SX_NO,          
}tReturnCodes;

extern volatile uint32_t TickCounter;

#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#endif /* __GNUC__ */

void BoardInit( void );            //板外围设备初始化

void Delay ( uint32_t delay );     //ms延时函数

void LongDelay ( uint8_t delay );  //s延时函数

uint32_t randr( uint32_t min, uint32_t max );  //计算最小值和最大值之间的随机数

#endif 
