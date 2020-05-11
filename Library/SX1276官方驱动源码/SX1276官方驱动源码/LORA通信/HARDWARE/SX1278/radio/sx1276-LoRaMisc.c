#include "platform.h"

#if defined( USE_SX1276_RADIO )

#include "sx1276-Hal.h"
#include "sx1276.h"
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"


#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

extern tLoRaSettings LoRaSettings;

//设置频率
void SX1276LoRaSetRFFrequency(uint32_t freq)    
{
    LoRaSettings.RFFrequency = freq;

    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276LR->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1276LR->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1276LR->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    SX1276WriteBuffer( REG_LR_FRFMSB, &SX1276LR->RegFrfMsb, 3 );
}

//得到频率
uint32_t SX1276LoRaGetRFFrequency( void )
{
    SX1276ReadBuffer( REG_LR_FRFMSB, &SX1276LR->RegFrfMsb, 3 );
    LoRaSettings.RFFrequency = ( ( uint32_t )SX1276LR->RegFrfMsb << 16 ) | ( ( uint32_t )SX1276LR->RegFrfMid << 8 ) | ( ( uint32_t )SX1276LR->RegFrfLsb );
    LoRaSettings.RFFrequency = ( uint32_t )( ( double )LoRaSettings.RFFrequency * ( double )FREQ_STEP );

    return LoRaSettings.RFFrequency;
}

//设置发射功率
//选择最大输出功率：Pmax=10.8+0.6 * power
void SX1276LoRaSetRFPower( int8_t power )
{
    SX1276Read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    SX1276Read( REG_LR_PADAC, &SX1276LR->RegPaDac );
    
    if( ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )  //如果选择了PA_BOOST，即功率不超过+20dBm
    {
        if( ( SX1276LR->RegPaDac & 0x87 ) == 0x87 )         //当OutputPower=1111 时，在PA_BOOST 上为+20dBm
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else                                               //如果默认值
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else                                                  //如果选择了RFO，即功率不超过14dBm
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
        SX1276LR->RegPaConfig = ( SX1276LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_LR_PACONFIG, SX1276LR->RegPaConfig );
    LoRaSettings.Power = power;
}


//得到发射功率
int8_t SX1276LoRaGetRFPower( void )
{
    SX1276Read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    SX1276Read( REG_LR_PADAC, &SX1276LR->RegPaDac );

    if( ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1276LR->RegPaDac & 0x07 ) == 0x07 )
        {
            LoRaSettings.Power = 5 + ( SX1276LR->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
        else
        {
            LoRaSettings.Power = 2 + ( SX1276LR->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
    }
    else
    {
        LoRaSettings.Power = -1 + ( SX1276LR->RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
    }
    return LoRaSettings.Power;
}

//设置带宽
void SX1276LoRaSetSignalBandwidth( uint8_t bw )
{
    SX1276Read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
    SX1276Write( REG_LR_MODEMCONFIG1, SX1276LR->RegModemConfig1 );
    LoRaSettings.SignalBw = bw;
}

//得到带宽
uint8_t SX1276LoRaGetSignalBandwidth( void )
{
    SX1276Read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    LoRaSettings.SignalBw = ( SX1276LR->RegModemConfig1 & ~RFLR_MODEMCONFIG1_BW_MASK ) >> 4;
    return LoRaSettings.SignalBw;
}

//设置扩频因子
void SX1276LoRaSetSpreadingFactor( uint8_t factor )
{

    if( factor > 12 )
    {
        factor = 12;
    }
    else if( factor < 6 )
    {
        factor = 6;
    }

    if( factor == 6 )
    {
        SX1276LoRaSetNbTrigPeaks( 5 );
    }
    else
    {
        SX1276LoRaSetNbTrigPeaks( 3 );
    }

    SX1276Read( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2 );    
    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
    SX1276Write( REG_LR_MODEMCONFIG2, SX1276LR->RegModemConfig2 );    
    LoRaSettings.SpreadingFactor = factor;
}

//得到扩频因子
uint8_t SX1276LoRaGetSpreadingFactor( void )
{
    SX1276Read( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2 );   
    LoRaSettings.SpreadingFactor = ( SX1276LR->RegModemConfig2 & ~RFLR_MODEMCONFIG2_SF_MASK ) >> 4;
    return LoRaSettings.SpreadingFactor;
}

void SX1276LoRaSetErrorCoding( uint8_t value )
{
    SX1276Read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
    SX1276Write( REG_LR_MODEMCONFIG1, SX1276LR->RegModemConfig1 );
    LoRaSettings.ErrorCoding = value;
}

uint8_t SX1276LoRaGetErrorCoding( void )
{
    SX1276Read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    LoRaSettings.ErrorCoding = ( SX1276LR->RegModemConfig1 & ~RFLR_MODEMCONFIG1_CODINGRATE_MASK ) >> 1;
    return LoRaSettings.ErrorCoding;
}

//CRC校验使能
void SX1276LoRaSetPacketCrcOn( bool enable )
{
    SX1276Read( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2 );
    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
    SX1276Write( REG_LR_MODEMCONFIG2, SX1276LR->RegModemConfig2 );
    LoRaSettings.CrcOn = enable;
}

//设置前导码长度
void SX1276LoRaSetPreambleLength( uint16_t value )
{
    SX1276ReadBuffer( REG_LR_PREAMBLEMSB, &SX1276LR->RegPreambleMsb, 2 );

    SX1276LR->RegPreambleMsb = ( value >> 8 ) & 0x00FF;
    SX1276LR->RegPreambleLsb = value & 0xFF;
    SX1276WriteBuffer( REG_LR_PREAMBLEMSB, &SX1276LR->RegPreambleMsb, 2 );
}

//得到前导码长度
uint16_t SX1276LoRaGetPreambleLength( void )
{
    SX1276ReadBuffer( REG_LR_PREAMBLEMSB, &SX1276LR->RegPreambleMsb, 2 );
    return ( ( SX1276LR->RegPreambleMsb & 0x00FF ) << 8 ) | SX1276LR->RegPreambleLsb;
}

//得到CRC校验是否开启
bool SX1276LoRaGetPacketCrcOn( void )
{
    SX1276Read( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2 );
    LoRaSettings.CrcOn = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON ) >> 1;
    return LoRaSettings.CrcOn;
}

//隐藏头部信息开关  默认关
void SX1276LoRaSetImplicitHeaderOn( bool enable )
{
    SX1276Read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    SX1276LR->RegModemConfig1 = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
    SX1276Write( REG_LR_MODEMCONFIG1, SX1276LR->RegModemConfig1 );
    LoRaSettings.ImplicitHeaderOn = enable;
}

//得到隐藏头部信息
bool SX1276LoRaGetImplicitHeaderOn( void )
{
    SX1276Read( REG_LR_MODEMCONFIG1, &SX1276LR->RegModemConfig1 );
    LoRaSettings.ImplicitHeaderOn = ( SX1276LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_ON );
    return LoRaSettings.ImplicitHeaderOn;
}

//使能单次接收模式
void SX1276LoRaSetRxSingleOn( bool enable )
{
    LoRaSettings.RxSingleOn = enable;
}

//得到是否开始单次接收模式
bool SX1276LoRaGetRxSingleOn( void )
{
    return LoRaSettings.RxSingleOn;
}

//使能跳频
void SX1276LoRaSetFreqHopOn( bool enable )
{
    LoRaSettings.FreqHopOn = enable;
}

//得到是否开始跳频
bool SX1276LoRaGetFreqHopOn( void )
{
    return LoRaSettings.FreqHopOn;
}

//设置跳频周期
void SX1276LoRaSetHopPeriod( uint8_t value )
{
    SX1276LR->RegHopPeriod = value;
    SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );
    LoRaSettings.HopPeriod = value;
}

//得到跳频周期
uint8_t SX1276LoRaGetHopPeriod( void )
{
    SX1276Read( REG_LR_HOPPERIOD, &SX1276LR->RegHopPeriod );
    LoRaSettings.HopPeriod = SX1276LR->RegHopPeriod;
    return LoRaSettings.HopPeriod;
}

//设置发送最大超时时间
void SX1276LoRaSetTxPacketTimeout( uint32_t value )
{
    LoRaSettings.TxPacketTimeout = value;
}

//得到发送最大超时时间
uint32_t SX1276LoRaGetTxPacketTimeout( void )
{
    return LoRaSettings.TxPacketTimeout;
}

//设置接收最大超时时间
void SX1276LoRaSetRxPacketTimeout( uint32_t value )
{
    LoRaSettings.RxPacketTimeout = value;
}

//得到接收最大超时时间
uint32_t SX1276LoRaGetRxPacketTimeout( void )
{
    return LoRaSettings.RxPacketTimeout;
}

//设置负载的长度（仅在隐藏头部开启有用）
void SX1276LoRaSetPayloadLength( uint8_t value )
{
    SX1276LR->RegPayloadLength = value;
    SX1276Write( REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength );
    LoRaSettings.PayloadLength = value;
}

//得到负载长度
uint8_t SX1276LoRaGetPayloadLength( void )
{
    SX1276Read( REG_LR_PAYLOADLENGTH, &SX1276LR->RegPayloadLength );
    LoRaSettings.PayloadLength = SX1276LR->RegPayloadLength;
    return LoRaSettings.PayloadLength;
}

//设置是否启动PA输出功率有+20dBm
void SX1276LoRaSetPa20dBm( bool enale )
{
    SX1276Read( REG_LR_PADAC, &SX1276LR->RegPaDac );    
    SX1276Read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    //如果选择PA输出引脚为RFO即RegPaConfig这个寄存器最高位为0，则输出功率不得超过+14dBm
	//如果选择PA输出引脚为PA_BOOST 即RegPaConfig这个寄存器最高位为1，则输出功率不超过+20dBm 
    //REG_LR_PADAC（在数据手册108页）这个寄存器可以设置 PA_BOOST 引脚上启动+20dBm 选项
    if( ( SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST ) //如果选择了PA_BOOST
    {    
        if( enale == true )
        {
            SX1276LR->RegPaDac = 0x87;     //当OutputPower=1111 时，在PA_BOOST 上为+20dBm
        }
    }
    else   //如果选择了RFO
    {
        SX1276LR->RegPaDac = 0x84;         //默认值
    }
    SX1276Write( REG_LR_PADAC, SX1276LR->RegPaDac );
}

//得到是否开启使能PA+20dBm
bool SX1276LoRaGetPa20dBm( void )
{
    SX1276Read( REG_LR_PADAC, &SX1276LR->RegPaDac );
    
    return ( ( SX1276LR->RegPaDac & 0x07 ) == 0x07 ) ? true : false;
}

//PA选择和输出功率控制
void SX1276LoRaSetPAOutput( uint8_t outputPin )
{
    SX1276Read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    SX1276LR->RegPaConfig = (SX1276LR->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | outputPin;
    SX1276Write( REG_LR_PACONFIG, SX1276LR->RegPaConfig );
}

//得到此时的引脚
uint8_t SX1276LoRaGetPAOutput( void )
{
    SX1276Read( REG_LR_PACONFIG, &SX1276LR->RegPaConfig );
    return SX1276LR->RegPaConfig & ~RFLR_PACONFIG_PASELECT_MASK;
}

//设置斜升斜降的时间
void SX1276LoRaSetPaRamp( uint8_t value )
{
    SX1276Read( REG_LR_PARAMP, &SX1276LR->RegPaRamp );
    SX1276LR->RegPaRamp = ( SX1276LR->RegPaRamp & RFLR_PARAMP_MASK ) | ( value & ~RFLR_PARAMP_MASK );
    SX1276Write( REG_LR_PARAMP, SX1276LR->RegPaRamp );
}

//得到斜升斜降的时间
uint8_t SX1276LoRaGetPaRamp( void )
{
    SX1276Read( REG_LR_PARAMP, &SX1276LR->RegPaRamp );
    return SX1276LR->RegPaRamp & ~RFLR_PARAMP_MASK;
}

//设置接收超时时间
void SX1276LoRaSetSymbTimeout( uint16_t value )
{
    SX1276ReadBuffer( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2, 2 );

    SX1276LR->RegModemConfig2 = ( SX1276LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    SX1276LR->RegSymbTimeoutLsb = value & 0xFF;
    SX1276WriteBuffer( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2, 2 );
}

//得到设置的接收超时时间
uint16_t SX1276LoRaGetSymbTimeout( void )
{
    SX1276ReadBuffer( REG_LR_MODEMCONFIG2, &SX1276LR->RegModemConfig2, 2 );
    return ( ( SX1276LR->RegModemConfig2 & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) << 8 ) | SX1276LR->RegSymbTimeoutLsb;
}

void SX1276LoRaSetLowDatarateOptimize( bool enable )
{
    SX1276Read( REG_LR_MODEMCONFIG3, &SX1276LR->RegModemConfig3 );
    SX1276LR->RegModemConfig3 = ( SX1276LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) | ( enable << 3 );
    SX1276Write( REG_LR_MODEMCONFIG3, SX1276LR->RegModemConfig3 );
}

bool SX1276LoRaGetLowDatarateOptimize( void )
{
    SX1276Read( REG_LR_MODEMCONFIG3, &SX1276LR->RegModemConfig3 );
    return ( ( SX1276LR->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON ) >> 3 );
}

//设置数据包长度
void SX1276LoRaSetNbTrigPeaks( uint8_t value )
{
    SX1276Read( 0x31, &SX1276LR->RegTestReserved31 );
    SX1276LR->RegTestReserved31 = ( SX1276LR->RegTestReserved31 & 0xF8 ) | value;//数据包长度最高有效位 0x31 bit2 1 0
    SX1276Write( 0x31, SX1276LR->RegTestReserved31 );
}

//得到数据包长度
uint8_t SX1276LoRaGetNbTrigPeaks( void )
{
    SX1276Read( 0x31, &SX1276LR->RegTestReserved31 );
    return ( SX1276LR->RegTestReserved31 & 0x07 );
}

#endif 
