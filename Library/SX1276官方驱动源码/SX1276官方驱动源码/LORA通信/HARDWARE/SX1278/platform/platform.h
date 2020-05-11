#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#ifndef __GNUC__
#define inline
#endif

//平台的定义
#define Bleeper                                     3
#define SX1243ska                                   2
#define SX12xxEiger                                 1
#define SX12000DVK                                  0

//平台的选择。
//请取消平台定义并选择您的平台，或在编译器定义选项上添加/更改平台定义
#define PLATFORM                                    SX12xxEiger

#if( PLATFORM == SX12xxEiger )     //如果PLATFORM == SX12xxEiger就定以下面的东西

	//lora模块型号的选择。
	//请取消对所需电台的注释，并对其他平台进行注释，或在编译器定义选项上添加/更改所需型号定义
	#define USE_SX1276_RADIO
	//#define USE_SX1232_RADIO
	//#define USE_SX1272_RADIO
	//#define USE_SX1243_RADIO

	//模块的选择。
	//SX1276现有三个模块。请将连接的模块设置为值1，并将其他模块设置为0
	#ifdef USE_SX1276_RADIO
		#define MODULE_SX1276RF1IAS                     0
		#define MODULE_SX1276RF1JAS                     0
		#define MODULE_SX1276RF1KAS                     1
	#endif

	#include "sx12xxEiger/sx12xxEiger.h"
	#define USE_UART                                     0

#elif( PLATFORM == SX12000DVK )   //如果PLATFORM == SX12000DVK就定以下面的东西

	#define USE_SX1276_RADIO
	//#define USE_SX1232_RADIO
	//#define USE_SX1272_RADIO
	//#define USE_SX1243_RADIO
	#include "sx1200dvk/sx1200dvk.h"

#elif( PLATFORM == SX1243ska )   //如果PLATFORM == SX1243ska

#elif( PLATFORM == Bleeper )     //如果PLATFORM == Bleeper就定以下面的东西
    #define USE_SX1272_RADIO
    
    #include "bleeper/bleeper.h"
    #define USE_UART                                       0

#else                         //如果PLATFORM没有定义
    #error "Missing define: Platform (ie. SX12xxEiger)"
#endif


void Soft_delay_us(u16 time);
void Soft_delay_ms(u16 time);


#endif 
