#include <string.h>

#include "platform.h"
#include "key.h"
#include "timer.h"
#if defined( USE_SX1276_RADIO )
#include "radio.h"
#include "sx1276-Hal.h"
#include "sx1276.h"
#include "sx1276-LoRaMisc.h"
#include "sx1276-LoRa.h"

#include "usart.h"

#define LoRa_FREQENCY                              470000000

//下面的值用来计算RSSI（信号强度）
#define RSSI_OFFSET_LF                              -155.0
#define RSSI_OFFSET_HF                              -150.0

#define NOISE_ABSOLUTE_ZERO                         -174.0
#define NOISE_FIGURE_LF                                4.0
#define NOISE_FIGURE_HF                                6.0 

extern u32 Tx_Time_Start,Tx_Time_End;   //记录发送时间用的时间
extern u32 Rx_Time_Start,Rx_Time_End;   //记录接收数据用的时间
//预先计算的信号带宽，用于计算RSSI值
const double SignalBwLog[] =
{
    3.8927900303521316335038277369285,  // 7.8 kHz
    4.0177301567005500940384239336392,  // 10.4 kHz
    4.193820026016112828717566631653,   // 15.6 kHz
    4.31875866931372901183597627752391, // 20.8 kHz
    4.4948500216800940239313055263775,  // 31.2 kHz
    4.6197891057238405255051280399961,  // 41.6 kHz
    4.795880017344075219145044421102,   // 62.5 kHz
    5.0969100130080564143587833158265,  // 125 kHz
    5.397940008672037609572522210551,   // 250 kHz
    5.6989700043360188047862611052755   // 500 kHz
};

//这些值需要试验测得
const double RssiOffsetLF[] =
{   
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
};

//这些值需要试验测得
const double RssiOffsetHF[] =
{  
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
};

//跳频数据表
const int32_t HoppingFrequencies[] =
{
    916500000,
    923500000,
    906500000,
    917500000,
    917500000,
    909000000,
    903000000,
    916000000,
    912500000,
    926000000,
    925000000,
    909500000,
    913000000,
    918500000,
    918500000,
    902500000,
    911500000,
    926500000,
    902500000,
    922000000,
    924000000,
    903500000,
    913000000,
    922000000,
    926000000,
    910000000,
    920000000,
    922500000,
    911000000,
    922000000,
    909500000,
    926000000,
    922000000,
    918000000,
    925500000,
    908000000,
    917500000,
    926500000,
    908500000,
    916000000,
    905500000,
    916000000,
    903000000,
    905000000,
    915000000,
    913000000,
    907000000,
    910000000,
    926500000,
    925500000,
    911000000,
};

//默认设置
tLoRaSettings LoRaSettings =
{
    LoRa_FREQENCY ,   // 频率为470MHZ
    20,               // 发射功率
    9,                // 信号频宽 [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,           
                      // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
					  /*带宽也表示传输速率，这里的带宽指的是频宽，
					    即频段的频率宽度，它决定了中心频率的上下
					    频率。比如中心频率 433MHZ,带宽是 2MHZ，
					    则通信的频谱（信道）的频率为432MHZ~434MHZ*/
    9,                // 扩频因子[6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
	                  /*扩频设置用的，扩频因子越大，信噪比越高，传输速率越低，但是传输距离也会变远*/   
	                  /*如果该值为6，则必须得打开下方的隐藏头部信息开关*/
    1,                // 纠错码 [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
					  /*即有效信号和整个报文（数据包）的比值*/
    true,             // CRC效验开关[0: OFF, 1: ON]
    false,            // 隐藏头部信息开关 [0: OFF, 1: ON]
    0,                // 接收单次模式\连续模式配置 [0: Continuous, 1 Single]
    0,                // 跳频模式开关 [0: OFF, 1: ON]              //跳频技术
    4,                // 跳频之间的周期长度
    100,              // 最大发送时间
    100,              // 最大接收时间
    128,              // 数据长度 (用于隐式头模式)
};


tSX1276LR* SX1276LR;       //SX1276 LoRa寄存器变量

static uint8_t RFBuffer[RF_BUFFER_SIZE];  //这个数组既是发送数据用的数组也是接收时用的数组


uint8_t RFLRState = RFLR_STATE_IDLE;  //空闲模式


static uint16_t RxPacketSize = 0;     //接收数据长度
static int8_t RxPacketSnrEstimate;    //接收数据信噪比变量
static double RxPacketRssiValue;      //接收数据时信号强度变量
static uint8_t RxGain = 1;            //信号放大增益
static uint32_t RxTimeoutTimer = 0;   //接收时间
static uint32_t PacketTimeout;        //最大接收时间
static uint16_t TxPacketSize = 0;     //发送数据长度

//初始化SX1276LoRa模式
void SX1276LoRaInit( void )
{
	RFLRState = RFLR_STATE_IDLE;                                     //设置为空闲模式

	SX1276LoRaSetDefaults();                                         //读版本信息
	
    //从SX1276寄存器REG_LR_OPMODE开始读，依次把寄存器值读进SX1276Regs这个数组里面
	SX1276ReadBuffer( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );     //读取所有寄存器
    
    //将设备设置为睡眠模式
    //SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
	
    SX1276LR->RegLna = RFLR_LNA_GAIN_G1;                             //LNA增益最大 默认也是最大 但为啥还设置？
    SX1276WriteBuffer( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );    //又把读取到的寄存器值又一次写进去，这是要干嘛？
	
    //射频的一些默认设置
    SX1276LoRaSetRFFrequency( LoRaSettings.RFFrequency );            //设置频率   默认435000000
    SX1276LoRaSetSpreadingFactor( LoRaSettings.SpreadingFactor );    //SF6只在隐式报头模式下工作  默认7
    SX1276LoRaSetErrorCoding( LoRaSettings.ErrorCoding );            //纠错码
    SX1276LoRaSetPacketCrcOn( LoRaSettings.CrcOn );                  //CRC效验开关
    SX1276LoRaSetSignalBandwidth( LoRaSettings.SignalBw );           //带宽    默认9
    SX1276LoRaSetImplicitHeaderOn( LoRaSettings.ImplicitHeaderOn );  //隐藏头部信息开关  默认关
	
    SX1276LoRaSetSymbTimeout(0x3FF);                                 //斜升斜降时间为500us
    SX1276LoRaSetPayloadLength( LoRaSettings.PayloadLength );        //设置数据长度为128（仅在隐藏头部开启有用）
    SX1276LoRaSetLowDatarateOptimize( true );                        //设置最大接收超时时间为2^8+1

    #if( ( MODULE_SX1276RF1IAS == 1 ) || ( MODULE_SX1276RF1KAS == 1 ) ) /*我不晓得用的SX1276是哪个模块*/
        if( LoRaSettings.RFFrequency > 860000000 )                   //如果频率超过860MHZ
        {
            SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_RFO );     //选择RFO管脚输出信号
            SX1276LoRaSetPa20dBm( false );                           //最大功率不超过14dBm
            LoRaSettings.Power = 14;                                 //发射功率为14
            SX1276LoRaSetRFPower( LoRaSettings.Power );              //设置发射功率 
        }
        else                                                         //如果设置功率小于860MHZ（显然我们用这是下面这种设置）
        {
			//SX1276Write( REG_LR_OCP, 0x3f );
            SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST ); //选择PA_BOOST管脚输出信号
            SX1276LoRaSetPa20dBm( true );                            //最大功率可达到+20dMb 
            LoRaSettings.Power = 20;                                 //发射功率为20
            SX1276LoRaSetRFPower( LoRaSettings.Power );              //设置发射功率        
        } 
    #elif( MODULE_SX1276RF1JAS == 1 )	
		if( LoRaSettings.RFFrequency > 380000000 )                   //如果频率超过380MHZ
		{
			SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST ); //选择PA_BOOST管脚输出信号
			SX1276LoRaSetPa20dBm( true );                            //最大功率可达到+20dMb
			LoRaSettings.Power = 20;                                 //发射功率为20
			SX1276LoRaSetRFPower( LoRaSettings.Power );              //设置发射功率   
		}
		else
		{
			SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_RFO );
			SX1276LoRaSetPa20dBm( false );
			LoRaSettings.Power = 14;
			SX1276LoRaSetRFPower( LoRaSettings.Power );
		} 
	#endif
    SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );	                      // 将设备设置为待机模式
}

//设置版本号
void SX1276LoRaSetDefaults( void )
{
    SX1276Read( REG_LR_VERSION, &SX1276LR->RegVersion );
}

//SX1276复位
//SX1276官方提供是时钟源即延时函数使用定时器设置的
void SX1276LoRaReset( void )
{
    uint32_t startTick;
	
	SX1276SetReset( RADIO_RESET_ON );
    
    startTick = GET_TICK_COUNT( );                                    //等待1ms   
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 1 ) );    

    SX1276SetReset( RADIO_RESET_OFF );
    
    startTick = GET_TICK_COUNT( );                                    //等待6ms
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 6 ) );    
}

/*****************************************************************
* 功能描述：	设置LoRa工作模式
* 入口参数：RFLR_OPMODE_SLEEP		         --	睡眠模式
			RFLR_OPMODE_STANDBY		         --	待机模式
            RFLR_OPMODE_SYNTHESIZER_TX       -- 频率合成发送模式
			RFLR_OPMODE_TRANSMITTER	         --	发送模式
            RFLR_OPMODE_SYNTHESIZER_RX       -- 频率合成接收模式
			RFLR_OPMODE_RECEIVER	         --	持续接收模式
            RFLR_OPMODE_RECEIVER_SINGLE      -- 单次接收模式
            RFLR_OPMODE_CAD                  -- 信号活动检测模式
* 返回值：		无
*******************************************************************/
void SX1276LoRaSetOpMode( uint8_t opMode )
{
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = true;
    bool antennaSwitchTxOn = false;                                  //天线开关状态
    opModePrev = SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;
    if( opMode != opModePrev )                                       //如果不是待机模式
    {
        if( opMode == RFLR_OPMODE_TRANSMITTER )                      //如果是发送模式
		{
			antennaSwitchTxOn = true;
		}
		else
		{
			antennaSwitchTxOn = false;
		}
		if( antennaSwitchTxOn != antennaSwitchTxOnPrev )
		{
			antennaSwitchTxOnPrev = antennaSwitchTxOn;
			RXTX( antennaSwitchTxOn );                                //收发模式切换 RXTX引脚没有使用 所以应该是软件切换
		}
		SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_MASK ) | opMode;
		//REG_LR_OPMODE寄存器查看数据手册111页
		SX1276Write( REG_LR_OPMODE, SX1276LR->RegOpMode );        
	}
}

//获取当前LORA的模式 
uint8_t SX1276LoRaGetOpMode( void )                       
{
    SX1276Read( REG_LR_OPMODE, &SX1276LR->RegOpMode );
    
    return SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;
}

//获取LNA（信号放大）增益（001=最大增益，010=最大增益-6dB，011=最大增益-12dB..........）
uint8_t SX1276LoRaReadRxGain( void )                    
{
	//REG_LR_LNA寄存器查看数据手册114页
	SX1276Read( REG_LR_LNA, &SX1276LR->RegLna );
	return( SX1276LR->RegLna >> 5 ) & 0x07;
}

//获取信号强度
double SX1276LoRaReadRssi( void )
{  
    SX1276Read( REG_LR_RSSIVALUE, &SX1276LR->RegRssiValue );

    if( LoRaSettings.RFFrequency < 860000000 )  
    {
        return RssiOffsetLF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegRssiValue;
    }
    else
    {
        return RssiOffsetHF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegRssiValue;
    }
}

//获取数据时的增益值
uint8_t SX1276LoRaGetPacketRxGain( void )
{
    return RxGain;
}

//获取数据时的信噪比值，信号和噪声的比值，信噪比越高，说明信号干扰越小。
int8_t SX1276LoRaGetPacketSnr( void )
{
    return RxPacketSnrEstimate;
}

//获取数据时的无线信号强度
double SX1276LoRaGetPacketRssi( void )
{
    return RxPacketRssiValue;
}

//开始接收
void SX1276LoRaStartRx( void )
{
    SX1276LoRaSetRFState( RFLR_STATE_RX_INIT );
}

//接收数据
void SX1276LoRaGetRxPacket( void *buffer, uint16_t *size )
{
	*size = RxPacketSize;
	RxPacketSize = 0;
	memcpy( (void*)buffer, (void*)RFBuffer, (size_t)*size );
}

//发送数据
void SX1276LoRaSetTxPacket( const void *buffer, uint16_t size )
{
    if( LoRaSettings.FreqHopOn == false )    //默认是关了的
    {
        TxPacketSize = size;
    }
    else
    {
        TxPacketSize = 255;
    }
    memcpy( ( void * )RFBuffer, buffer, ( size_t )TxPacketSize );  //RFBuffer是个装发送数据的数组，而buffer是你要发送的数据的数组

    RFLRState = RFLR_STATE_TX_INIT;                                //设置状态为发送初始化，只要设置了RFLR_STATE_TX_INIT，就会开始在SX1276LoRaProcess（）函数中发送数据      
}

//得到RFLRState状态
uint8_t SX1276LoRaGetRFState( void )
{
    return RFLRState;
}

//设置RFLRState状态，RFLRState的值决定了下面的函数处理哪一步的代码
void SX1276LoRaSetRFState( uint8_t state )
{
    RFLRState = state;
}

/*
    发送和接收的处理函数

	返回值： RF_BUSY,               //繁忙状态
             RFLR_STATE_IDLE        //空闲状态
             RFLR_STATE_RX_INIT     //接收初始化状态
             RFLR_STATE_RX_RUNNING  //正在接收状态
			 RFLR_STATE_RX_DON，    //接收完成
			 RFLR_STATE_RX_TIMEOUT  //接收超时 
			 RFLR_STATE_TX_INIT     //发送初始化状态
			 RFLR_STATE_TX_RUNNING  //正在发送状态
			 RFLR_STATE_TX_DONE     //发送完成
             RFLR_STATE_CAD_RUNNIN，// 
			 RFLR_STATE_CAD_INI，
			 RFLR_STATE_TX_DONE                                    
 */
uint32_t SX1276LoRaProcess( void )
{
    uint32_t result = RF_BUSY;
    uint8_t regValue=0;
    switch( RFLRState )
    {
		case RFLR_STATE_IDLE:                                                 //空闲状态即返回
             break;
        case RFLR_STATE_RX_INIT:                                               //开启了一些中断，设置跳频周期，设置端口映射，设置接收模式，设置最大接收时间
			 SX1276LoRaSetOpMode(RFLR_OPMODE_STANDBY);                         //待机模式
		     //REG_LR_IRQFLAGSMASK寄存器看数据手册115页
		     SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT          |    
									   //RFLR_IRQFLAGS_RXDONE             |    //开启接收完成中断
									     RFLR_IRQFLAGS_PAYLOADCRCERROR    |    //开启负载CRC错误中断 
									     RFLR_IRQFLAGS_VALIDHEADER        |
									     RFLR_IRQFLAGS_TXDONE             |
									     RFLR_IRQFLAGS_CADDONE            |
                                         RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |    //开启FHSS改变信道中断
									     RFLR_IRQFLAGS_CADDETECTED;
		     SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );    

		     if(LoRaSettings.FreqHopOn == true )                               //默认为false
		     {
		         SX1276LR->RegHopPeriod = LoRaSettings.HopPeriod;              //默认为0   
		       //REG_LR_HOPCHANNEL寄存器看数据手册115页				
                 SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );    //获取使用中的调频当前信道值 
			     SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
		     }
		     else    
		     { 
                 SX1276LR->RegHopPeriod = 255;
		     } 
		     //REG_LR_HOPPERIOD寄存器看数据手册118页
		     SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );  
			
		     //端口映射设置			
                                        // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
		     SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                        // CadDetected               ModeReady
		     SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_10 | RFLR_DIOMAPPING2_DIO5_00;
		
		     SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );
			
		     if( LoRaSettings.RxSingleOn == true )                               //单次接收模式,默认持续接收模式
		     {
			     SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
		     }
	         else                                                                //持续接收模式
		     {
			     SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;
			     //REG_LR_FIFOADDRPTR寄存器查看数据寄存器113页
			     SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );    
			     SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );
				 
		     }   
		     memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );                    //清空RFBuffer
             Rx_Time_Start=TickCounter;
		     PacketTimeout = LoRaSettings.RxPacketTimeout;                       //设置最大接收时间为100
		     RxTimeoutTimer = GET_TICK_COUNT( );
		     RFLRState = RFLR_STATE_RX_RUNNING;
		     break;
		case RFLR_STATE_RX_RUNNING:                                              //检测发生了那些中断（譬如接受完成），并且清除中断标志位，计算是否接收超时  
			 SX1276Read(0x12,&regValue);                                         //读取此时开启了哪些中断  
             //if( DIO0 == 1 )                                                   //不用DIO 就读取寄存器标志  
			 if(regValue & (1<<6))                                               //接收完成
             {
				 //printf("接收成功！\r\n");
		         RxTimeoutTimer = GET_TICK_COUNT( );
                 if( LoRaSettings.FreqHopOn == true )                             //默认false
                 { 
				     SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
					 SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
				 }
        
				 SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );           //清除标志位
				 RFLRState = RFLR_STATE_RX_DONE;
			  }
			  //if( DIO2 == 1 )                                                   //不用DIO 就读取寄存器标志
			  if(regValue & (1<<1))                                               //FHSS改变信道
			  {
			      RxTimeoutTimer = GET_TICK_COUNT( );
			      if( LoRaSettings.FreqHopOn == true )
				  {
					  SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
					  SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
				  }
        
				  SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL ); //清除标志位      
				  //RxGain = SX1276LoRaReadRxGain( );                               //获取LNA增益  LNA：这个是把天线接受到的信号进行放大，为解码做准备的
			  }
			  if( LoRaSettings.RxSingleOn == true )                                 //单次接收模式，默认持续接受模式
			  {
			     if( ( GET_TICK_COUNT( ) - RxTimeoutTimer ) > PacketTimeout )       //计算是否超时
				 {
				      RFLRState = RFLR_STATE_RX_TIMEOUT;                            //接收超时 
				 }
			 }
             break;
		case RFLR_STATE_RX_DONE:                                                    //对接收数据进行CRC检测，得到接收数据时的信噪比，并且计算信号强度，
			                                                                        //获取接收数据的字节长度，并且在SX1276的fifo中读取数据（因为此时
			 																		//对方传来的数据被SX1276接收到，并储存在fifo中）
		     SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
			 if( ( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )   //CRC校验
			 {      
				 SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );     //清除标志位   
				 if( LoRaSettings.RxSingleOn == true )                              //单次接收模式
				 {
					RFLRState = RFLR_STATE_RX_INIT;                                
				 }
				 else
				 { 
					RFLRState = RFLR_STATE_RX_RUNNING;
				 }
				 break;
		     }
		     //以下代码可用可不用，主要是为了得到信号的强度和信噪比	
/*			 {
				 uint8_t rxSnrEstimate;
				 //REG_LR_PKTSNRVALUE寄存器查看数据寄存器116页   
				 SX1276Read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );                     //最后接收到的数据包的 SNR 预估值
				 if( rxSnrEstimate & 0x80 )  //说明是负数                              // The SNR sign bit is 1
				 {

					 RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;     //除以4
					 RxPacketSnrEstimate = -RxPacketSnrEstimate;
				 }
				 else
				 {
					 RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
				 }
			}        
			if( LoRaSettings.RFFrequency < 860000000 ) 
			{    
				if( RxPacketSnrEstimate < 0 )          //SNR即信噪比小于0时
				{
					//单个包的信号强度，下面是计算公式，高频和低频信号有点不同，SNR值的正负也会不同
					RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_LF + ( double )RxPacketSnrEstimate;
				}
				else
				{    
					SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
					RxPacketRssiValue = RssiOffsetLF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegPktRssiValue;
				}
			}
			else                                        
			{    
				if( RxPacketSnrEstimate < 0 )
				{
					RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_HF + ( double )RxPacketSnrEstimate;
				}
				else
				{    
					SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
					RxPacketRssiValue = RssiOffsetHF[LoRaSettings.SignalBw] + ( double )SX1276LR->RegPktRssiValue;
				}
			}*/
			if( LoRaSettings.RxSingleOn == true )                                     //单次接收模式，默认持续模式
			{
			    SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;       
				SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );          //读取接收数据
			    if( LoRaSettings.ImplicitHeaderOn == true )                           //默认false
				{
				    RxPacketSize = SX1276LR->RegPayloadLength;
				    SX1276ReadFifo( RFBuffer, SX1276LR->RegPayloadLength );           //在Lora中读取接收到的数据
				}
				else
				{
					//REG_LR_NBRXBYTES寄存器查看数据寄存器115页  最近一次接收到的数据包的负载字节数
				    SX1276Read( REG_LR_NBRXBYTES, &SX1276LR->RegNbRxBytes );
					RxPacketSize = SX1276LR->RegNbRxBytes;
				    SX1276ReadFifo( RFBuffer, SX1276LR->RegNbRxBytes );
			     }
			 }
			 else                                                                     //接续接收模式
			 {
			     //REG_LR_FIFORXCURRENTADDR寄存器查看数据寄存器115页   接收到最后一个数据包的起始地址（数据缓冲区中）
				 SX1276Read( REG_LR_FIFORXCURRENTADDR, &SX1276LR->RegFifoRxCurrentAddr );
				 if( LoRaSettings.ImplicitHeaderOn == true )
				 {
					 RxPacketSize = SX1276LR->RegPayloadLength;
					 SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
					 SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );
				     SX1276ReadFifo( RFBuffer, SX1276LR->RegPayloadLength );
				 }
				 else
				 {
					 SX1276Read( REG_LR_NBRXBYTES, &SX1276LR->RegNbRxBytes );
					 RxPacketSize = SX1276LR->RegNbRxBytes;
					 SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
					 SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );
					 SX1276ReadFifo( RFBuffer, SX1276LR->RegNbRxBytes );
				 }
			 }  
			 if( LoRaSettings.RxSingleOn == true )                                     //单次接收模式
			 { 
				 RFLRState = RFLR_STATE_RX_INIT;
			 }
			 else                                                                      //接续接收模式
			 { 
				 RFLRState = RFLR_STATE_RX_RUNNING;
			 }
			 Rx_Time_End=TickCounter;
			 result = RF_RX_DONE;
			 break;
		case RFLR_STATE_RX_TIMEOUT:                                                   //接收时间超时
		     RFLRState = RFLR_STATE_RX_INIT;                                          //又重新开始接收，从初始化开始
		     result = RF_RX_TIMEOUT;
		     break;
		case RFLR_STATE_TX_INIT:                                                      //发送初始化，开启发送完成中断，
																					  //设置调频周期为0，设置发送数据的长度，设置为发送模式
		     Tx_Time_Start=TickCounter;
			 SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );                              //待机模式
			 if( LoRaSettings.FreqHopOn == true )                                     //跳频,默认false
			 {
				 SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT          |
											 RFLR_IRQFLAGS_RXDONE             |
											 RFLR_IRQFLAGS_PAYLOADCRCERROR    |
											 RFLR_IRQFLAGS_VALIDHEADER        |
										   //RFLR_IRQFLAGS_TXDONE             |       //开启发送完成中断
											 RFLR_IRQFLAGS_CADDONE            |
										   //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |       //开启FHSS改变信道中断
											 RFLR_IRQFLAGS_CADDETECTED;
				 SX1276LR->RegHopPeriod = LoRaSettings.HopPeriod;
				 SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
				 SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
			 }
			 else
			 {
				 //REG_LR_IRQFLAGSMASK寄存器看数据手册115
				 SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT          |
									         RFLR_IRQFLAGS_RXDONE             |
											 RFLR_IRQFLAGS_PAYLOADCRCERROR    |
											 RFLR_IRQFLAGS_VALIDHEADER        |
										   //RFLR_IRQFLAGS_TXDONE             |      //开启发送完成中断
											 RFLR_IRQFLAGS_CADDONE            |
											 RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
											 RFLR_IRQFLAGS_CADDETECTED;             
				 SX1276LR->RegHopPeriod = 0;
			 }
			 SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );                //设置跳频周期，默认为0
			 SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );      

			 SX1276LR->RegPayloadLength = TxPacketSize;                              //初始化负载大小
			 SX1276Write( REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength );
			
			 SX1276LR->RegFifoTxBaseAddr = 0x00;                                     
			 SX1276Write( REG_LR_FIFOTXBASEADDR, SX1276LR->RegFifoTxBaseAddr );

			 SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoTxBaseAddr;
			 SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );
			  
			 SX1276WriteFifo( RFBuffer, SX1276LR->RegPayloadLength );                //将要发送的数组写到LORA的fifo中
			 								// TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
			 SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
											// PllLock              Mode Ready
			 SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
			
			 SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

			 SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );

			 RFLRState = RFLR_STATE_TX_RUNNING;
			 break;
		case RFLR_STATE_TX_RUNNING:                                            //发送状态，检测发生那些中断（譬如发送完成中断）
			 SX1276Read(0x12,&regValue);
		     //if( DIO0 == 1 )                                                 //发送完成
			 if(regValue & (1<<3))
			 {
		         printf("发送成功！\r\n");
				 SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );          //清除标志位
				 RFLRState = RFLR_STATE_TX_DONE;   
			 }	
			 //if( DIO2 == 1 )                                                  //FHSS改变信道
			 if(regValue & (1<<3))
			 {
				 if( LoRaSettings.FreqHopOn == true )
				 {
					 SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
					 SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
				 }				
				 SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL ); //清除标志位
			 }
			 break;
		case RFLR_STATE_TX_DONE:                                                //发送完成 又回到空闲状态 
             Tx_Time_End=TickCounter;			
			 SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );                         //通过在数据包发送后立即关闭发射机来优化功耗
			 RFLRState = RFLR_STATE_IDLE;
			 result = RF_TX_DONE;
			 break;
		case RFLR_STATE_CAD_INIT:                                               //CAD信号活动检测
			 SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
			 SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT           |
										 RFLR_IRQFLAGS_RXDONE              |
										 RFLR_IRQFLAGS_PAYLOADCRCERROR     |
										 RFLR_IRQFLAGS_VALIDHEADER         |
										 RFLR_IRQFLAGS_TXDONE              |
									   //RFLR_IRQFLAGS_CADDONE             |
										 RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; 
									   //RFLR_IRQFLAGS_CADDETECTED;
			 SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );
			   
										 // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
			 SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
										 // CAD Detected              ModeReady
			 SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
			 SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );
				
			 SX1276LoRaSetOpMode( RFLR_OPMODE_CAD );
			 RFLRState = RFLR_STATE_CAD_RUNNING;
			 break;
		case RFLR_STATE_CAD_RUNNING:
			 if( DIO3 == 1 ) //CAD Done interrupt
			 { 
				SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE  );
				if( DIO4 == 1 ) // CAD Detected interrupt
				{
					SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED  );
					//CAD detected, we have a LoRa preamble
					RFLRState = RFLR_STATE_RX_INIT;
					result = RF_CHANNEL_ACTIVITY_DETECTED;
                } 
				else
				{    
					// The device goes in Standby Mode automatically    
					RFLRState = RFLR_STATE_IDLE;
					result = RF_CHANNEL_EMPTY;
				}
			 }   
			 break;			
        default:
             break;
    } 
    return result;
}












/*******************************************************************************/
//因为上面的处理函数直接一大段很复杂，所以下面我简化了下，其实就是分成了几个函数
/*******************************************************************************/

/********************************************************************
	* 功能描述：	获取调用本函数前最后接到那包数据的信噪比
	* 入口参数：	无
	* 返回值：		信噪比，有正有负
*********************************************************************/
int8_t GetLoRaSNR (void)                                                //512到526行有这部分代码
{
	u8 rxSnrEstimate;
	//取最后一包数据的信噪比         									//记录信噪比估值
	SX1276Read(REG_LR_PKTSNRVALUE, &rxSnrEstimate );					//信噪比
	//The SNR sign bit is 1												//信噪比小于0的情况，负数
	if( rxSnrEstimate & 0x80 )
	{
		RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;	//Invert and divide by 4
		RxPacketSnrEstimate = -RxPacketSnrEstimate;
	}
	else
	{
		//信噪比大于0的情况，正数
		RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;			//Divide by 4
	}
	return RxPacketSnrEstimate;
}

//得到信号增益，即LNA
uint8_t GetLoRaLNA()
{
    return SX1276LoRaReadRxGain();
}

/*******************************************************************************
	* 功能描述:     获取调用本函数前最后接到那包数据的信号强度
	* 入口参数:     无
	* 返回值:       信号强度，有正有负
********************************************************************************/
double GetPackLoRaRSSI(void)                                             //527行到551行
{
	if( GetLoRaSNR() < 0 )
	{
		/***************************************************************************************
			*功率：P=-174（dBm） + BW(dB) + NF(dB) + SNR(dB);
			*在信号被噪声淹没的情况下用此公式反推RSSI，前三项是热噪声功率,最后一项是信噪比
		****************************************************************************************/
		//信号被噪声淹没
		RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + \
		                    NOISE_FIGURE_LF     + (double)RxPacketSnrEstimate;
	}
	else
	{
		//信号强于噪声
		SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
		RxPacketRssiValue = RssiOffsetLF[LoRaSettings.SignalBw] + (double)SX1276LR->RegPktRssiValue;
	}
	return RxPacketRssiValue;
}

/*********************************************************************************************************
	* 功能描述:      读取当前信号强度,返回的是调用这个函数那一刻的信号强度需要在接收状态下调用才有用。
	* 入口参数:      无
	* 返回值:        无
*********************************************************************************************************/
//double SX1276LoRaReadRssi( void )这函数上面有定义，去上面看就行


/*********************************************************************************************************
	* 功能描述:      进入接收状态
	* 入口参数:      无
	* 返回值:        无
*********************************************************************************************************/
void LoRaRxStateEnter (void)
{
	//1.---->进入待机模式
	SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );		//操作寄存器只能在在Standby,sleep or FSTX 模式

	//2.---->接收初始化，参数设置
	SX1276LR->RegIrqFlagsMask	= 	RFLR_IRQFLAGS_RXTIMEOUT          |	//超时中断屏蔽
	                              //RFLR_IRQFLAGS_RXDONE             |	//打开数据包接收完成中断
	                              //RFLR_IRQFLAGS_PAYLOADCRCERROR    |	//负载CRC错误中断屏蔽
	                                RFLR_IRQFLAGS_VALIDHEADER        |	//Rx模式下接收到的有效报头屏蔽
	                                RFLR_IRQFLAGS_TXDONE             |	//FIFO负载发送完成中断屏蔽
	                                RFLR_IRQFLAGS_CADDONE            |	//CAD完成中断屏蔽
	                                RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |	//FHSS改变信道中断屏蔽
	                                RFLR_IRQFLAGS_CADDETECTED;			//检测到CAD中断屏蔽
	SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );		//设置需要屏蔽的中断(被注释掉的中断即打开)

	SX1276LR->RegHopPeriod = 255;
	SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );			//频率跳变之间的符号周期:255

	//DIO0:RxDone
	SX1276LR->RegDioMapping1 = 	RFLR_DIOMAPPING1_DIO0_00 |				//DIO0~DIO3引脚映射
	                            RFLR_DIOMAPPING1_DIO1_11 |
	                            RFLR_DIOMAPPING1_DIO2_11 |
	                            RFLR_DIOMAPPING1_DIO3_11;

	SX1276LR->RegDioMapping2 = 	RFLR_DIOMAPPING2_DIO4_10 | 				//DIO4~DIO5引脚映射
	                            RFLR_DIOMAPPING2_DIO5_11;

	/*****************************************************************************
		*在ZM470SX-M上,DIO4接到射频开关的6脚,进入接收状态后此引脚需要拉高,
		*所以在LoRa模式必须设为:RFLR_DIOMAPPING2_DIO4_10
	******************************************************************************/
	SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );	//配置5个IO的功能

	//3.---->设置接收模式(单次/持续模式)
	if( LoRaSettings.RxSingleOn == true )
	{
		//设置为单次接收
		SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
	}
	else
	{
		//设置为持续接收模式
		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;		  	//内容:0x00
		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );	//SPI访问FIFO的地址
		SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );           			//进入接收状态
	}
//	PacketTimeout  = LoRaSettings.RxPacketTimeout;						//超时时间为 100
//	RxTimeoutTimer = GET_TICK_COUNT( );									//接收超时时间为 10
}

/**************************************************************
	* 功能描述：   读取接收到数据
	* 入口参数：	pbuf	--	数据缓冲区指针
					size	--	数据字节长度指针
	* 返回值：		无
***************************************************************/
void LoRaRxDataRead (u8 *pbuf, u8 *size )
{
	if( DIO0 == 1 ) 
	{                                                  				//RxDone  接收完成                    
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );		//执行写操作以清除接收中断标志
	}	
	
    SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );			//读取中断状态标志位    
	//检测CRC标志，以验证数据包的完整性 	
	if( ( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
	{
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );	//执行写操作以清除接收中断标志
		*size = 0;
		return;
	}     
	
//	GetLoRaSNR();                  //读取最后一包数据信噪比(如果不关心这SNR参数可把此函数删除)     	     
//	GetPackLoRaRSSI();             //读取最后一包数据信号强度(如果不关心这RSSI参数可把此函数删除)

	/************* 取数据 ************/
	SX1276Read( REG_LR_FIFORXCURRENTADDR, &SX1276LR->RegFifoRxCurrentAddr );  	//读取最后接收一包数据首指针
	if( LoRaSettings.ImplicitHeaderOn == true ) 
	{
		RxPacketSize = SX1276LR->RegPayloadLength;
		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );			//写入FIFO的访问地址
		SX1276ReadFifo( RFBuffer, SX1276LR->RegPayloadLength );
	} 
	//显示报头模式
	else 
	{
		SX1276Read( REG_LR_NBRXBYTES, size );	 						//读取包长度(size记录数据包的长度)
//		RxPacketSize = SX1276LR->RegNbRxBytes;							//得到接收包的大小
		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;		//将最后接收的数据包的首地址赋予FIFO数据缓冲区的访问地址(SPI)
		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );  	//写入FIFO的访问地址
		SX1276ReadFifo( pbuf, *size);									//读取FIFO中的数据，并保存在pbuf中
	}
}	

u16 Tx_Counter = 0;
/****************************************************************
	* 功能描述：	发送数据
	* 入口参数：	pbuf		--	数据缓冲区指针
					size		--	数据字节数
	* 返回值：		0			--	发送成功
					其他(1,2)	--	发送失败
******************************************************************/
u8 LoRaTxData (u8 *pbuf, u8 size ,u8 *pcrcbuf ,u8 crcflag)
{
     u32 i,j; 
	if (0 == pbuf || 0 == size)	//没有数据可发送
	{
		return 1;				//发送失败 -- 返回1
	}		 
	
    SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );

	SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT          |
								RFLR_IRQFLAGS_RXDONE             |
								RFLR_IRQFLAGS_PAYLOADCRCERROR    |
								RFLR_IRQFLAGS_VALIDHEADER        |
								//RFLR_IRQFLAGS_TXDONE           |		//打开发送完成中断(Tx)
								RFLR_IRQFLAGS_CADDONE            |
								RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
								RFLR_IRQFLAGS_CADDETECTED;
	SX1276LR->RegHopPeriod = 0;											//FHSS跳频周期为0
	SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );
	SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );		//写入中断标志位

	//Initializes the payload size
	SX1276LR->RegPayloadLength = size; 									//数据负载长度(不包括CRC长度)
	SX1276Write( REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength +4); 	//在implicit模式(隐式包头),必须写入FIFO长度,0x80

	SX1276LR->RegFifoTxBaseAddr = 0x00;                                 //Full buffer used for Tx     
	SX1276Write( REG_LR_FIFOTXBASEADDR, SX1276LR->RegFifoTxBaseAddr );  //写入发送的首地址 

	SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoTxBaseAddr;
	SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );        //写入FIFO的访问地址
	SX1276WriteFifo( pbuf, SX1276LR->RegPayloadLength );	            //写入数疽要发送的数据
	//SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );      //写入FIFO的访问地址
	
	//写入CRC校验值
	if(crcflag==1)
	{
		if( size<(256-4) )
			SX1276WriteFifo( pcrcbuf, 4 );
		else
			return 1;
	}
	
	/*****************************************************************************
		*在ZM470SX-M上,DIO4接到射频开关的6脚,进入发送状态后此引脚需要拉低,
		*所以在LoRa模式必须设为:RFLR_DIOMAPPING2_DIO4_00
	******************************************************************************/
    //DIO0:TxDone  端口映射设置 
	SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_11| 
	                           RFLR_DIOMAPPING1_DIO2_11 | RFLR_DIOMAPPING1_DIO3_11;
	SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_11;
	SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );
	///////////////////////////////////////////////////////////////////////////////
    SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );		//进入发送状态 
	
	
    printf("开始发送时间为		--	t1:%d\r\n",TickCounter);
	i = 80000; 
	while ( DIO0 == 0 && i != 0)		//等待发送完成或者发送失败超时,完成后:DIO0为高电平
	{                 
        i--;	
		for(j = 0; j < 100; j++);		//超时等待约1ms        
	}
	
	printf("i的值为 -- %d！\r\n",80000-i);
	//如果发送失败   
	if (i == 0) 
	{   		
		printf("发送失败 -- 2\r\n");
		return 2;	//失败 -- 返回2
	} 
	//如果发送成功
	else 
	{					               
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );	//清除发送完成中断           
		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );				//进入待机模式
		printf("成功发送完成时间为	--	t2:%d\r\n",TickCounter);	
		printf("发送成功 -- 0\r\n");
		return 0;												//成功 -- 返回0
	}
	//SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );			//进入发送状态            
}

/******************************************************
	* 功能描述：    设置发射功率
	* 入口参数：    pwr: 功率范围5 ~ 20
	* 返回值：      无
*******************************************************/
void LoRaTxPower (u8 pwr)
{
	//默认参数PA最在电流为100mA,当输出20dBm时需要120mA,所以必须配置0x0b存器
	SX1276Write( REG_LR_OCP, 0x3f );
	SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST );
	SX1276LoRaSetPa20dBm( true );
	SX1276LoRaSetRFPower( pwr);
}

/******************************************************
	* 功能描述：	设置载波频率
	* 入口参数：    420000000~500000000
	* 返回值：      无
*******************************************************/
void LoRaFreqSet (u32 freq )
{
	SX1276LoRaSetRFFrequency( freq );
}
#endif 

