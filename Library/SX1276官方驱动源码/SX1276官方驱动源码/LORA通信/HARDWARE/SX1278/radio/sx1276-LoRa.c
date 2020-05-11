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

//�����ֵ��������RSSI���ź�ǿ�ȣ�
#define RSSI_OFFSET_LF                              -155.0
#define RSSI_OFFSET_HF                              -150.0

#define NOISE_ABSOLUTE_ZERO                         -174.0
#define NOISE_FIGURE_LF                                4.0
#define NOISE_FIGURE_HF                                6.0 

extern u32 Tx_Time_Start,Tx_Time_End;   //��¼����ʱ���õ�ʱ��
extern u32 Rx_Time_Start,Rx_Time_End;   //��¼���������õ�ʱ��
//Ԥ�ȼ�����źŴ������ڼ���RSSIֵ
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

//��Щֵ��Ҫ������
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

//��Щֵ��Ҫ������
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

//��Ƶ���ݱ�
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

//Ĭ������
tLoRaSettings LoRaSettings =
{
    LoRa_FREQENCY ,   // Ƶ��Ϊ470MHZ
    20,               // ���书��
    9,                // �ź�Ƶ�� [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,           
                      // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
					  /*����Ҳ��ʾ�������ʣ�����Ĵ���ָ����Ƶ��
					    ��Ƶ�ε�Ƶ�ʿ�ȣ�������������Ƶ�ʵ�����
					    Ƶ�ʡ���������Ƶ�� 433MHZ,������ 2MHZ��
					    ��ͨ�ŵ�Ƶ�ף��ŵ�����Ƶ��Ϊ432MHZ~434MHZ*/
    9,                // ��Ƶ����[6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
	                  /*��Ƶ�����õģ���Ƶ����Խ�������Խ�ߣ���������Խ�ͣ����Ǵ������Ҳ���Զ*/   
	                  /*�����ֵΪ6�������ô��·�������ͷ����Ϣ����*/
    1,                // ������ [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
					  /*����Ч�źź��������ģ����ݰ����ı�ֵ*/
    true,             // CRCЧ�鿪��[0: OFF, 1: ON]
    false,            // ����ͷ����Ϣ���� [0: OFF, 1: ON]
    0,                // ���յ���ģʽ\����ģʽ���� [0: Continuous, 1 Single]
    0,                // ��Ƶģʽ���� [0: OFF, 1: ON]              //��Ƶ����
    4,                // ��Ƶ֮������ڳ���
    100,              // �����ʱ��
    100,              // ������ʱ��
    128,              // ���ݳ��� (������ʽͷģʽ)
};


tSX1276LR* SX1276LR;       //SX1276 LoRa�Ĵ�������

static uint8_t RFBuffer[RF_BUFFER_SIZE];  //���������Ƿ��������õ�����Ҳ�ǽ���ʱ�õ�����


uint8_t RFLRState = RFLR_STATE_IDLE;  //����ģʽ


static uint16_t RxPacketSize = 0;     //�������ݳ���
static int8_t RxPacketSnrEstimate;    //������������ȱ���
static double RxPacketRssiValue;      //��������ʱ�ź�ǿ�ȱ���
static uint8_t RxGain = 1;            //�źŷŴ�����
static uint32_t RxTimeoutTimer = 0;   //����ʱ��
static uint32_t PacketTimeout;        //������ʱ��
static uint16_t TxPacketSize = 0;     //�������ݳ���

//��ʼ��SX1276LoRaģʽ
void SX1276LoRaInit( void )
{
	RFLRState = RFLR_STATE_IDLE;                                     //����Ϊ����ģʽ

	SX1276LoRaSetDefaults();                                         //���汾��Ϣ
	
    //��SX1276�Ĵ���REG_LR_OPMODE��ʼ�������ΰѼĴ���ֵ����SX1276Regs�����������
	SX1276ReadBuffer( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );     //��ȡ���мĴ���
    
    //���豸����Ϊ˯��ģʽ
    //SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
	
    SX1276LR->RegLna = RFLR_LNA_GAIN_G1;                             //LNA������� Ĭ��Ҳ����� ��Ϊɶ�����ã�
    SX1276WriteBuffer( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );    //�ְѶ�ȡ���ļĴ���ֵ��һ��д��ȥ������Ҫ���
	
    //��Ƶ��һЩĬ������
    SX1276LoRaSetRFFrequency( LoRaSettings.RFFrequency );            //����Ƶ��   Ĭ��435000000
    SX1276LoRaSetSpreadingFactor( LoRaSettings.SpreadingFactor );    //SF6ֻ����ʽ��ͷģʽ�¹���  Ĭ��7
    SX1276LoRaSetErrorCoding( LoRaSettings.ErrorCoding );            //������
    SX1276LoRaSetPacketCrcOn( LoRaSettings.CrcOn );                  //CRCЧ�鿪��
    SX1276LoRaSetSignalBandwidth( LoRaSettings.SignalBw );           //����    Ĭ��9
    SX1276LoRaSetImplicitHeaderOn( LoRaSettings.ImplicitHeaderOn );  //����ͷ����Ϣ����  Ĭ�Ϲ�
	
    SX1276LoRaSetSymbTimeout(0x3FF);                                 //б��б��ʱ��Ϊ500us
    SX1276LoRaSetPayloadLength( LoRaSettings.PayloadLength );        //�������ݳ���Ϊ128����������ͷ���������ã�
    SX1276LoRaSetLowDatarateOptimize( true );                        //���������ճ�ʱʱ��Ϊ2^8+1

    #if( ( MODULE_SX1276RF1IAS == 1 ) || ( MODULE_SX1276RF1KAS == 1 ) ) /*�Ҳ������õ�SX1276���ĸ�ģ��*/
        if( LoRaSettings.RFFrequency > 860000000 )                   //���Ƶ�ʳ���860MHZ
        {
            SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_RFO );     //ѡ��RFO�ܽ�����ź�
            SX1276LoRaSetPa20dBm( false );                           //����ʲ�����14dBm
            LoRaSettings.Power = 14;                                 //���书��Ϊ14
            SX1276LoRaSetRFPower( LoRaSettings.Power );              //���÷��书�� 
        }
        else                                                         //������ù���С��860MHZ����Ȼ���������������������ã�
        {
			//SX1276Write( REG_LR_OCP, 0x3f );
            SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST ); //ѡ��PA_BOOST�ܽ�����ź�
            SX1276LoRaSetPa20dBm( true );                            //����ʿɴﵽ+20dMb 
            LoRaSettings.Power = 20;                                 //���书��Ϊ20
            SX1276LoRaSetRFPower( LoRaSettings.Power );              //���÷��书��        
        } 
    #elif( MODULE_SX1276RF1JAS == 1 )	
		if( LoRaSettings.RFFrequency > 380000000 )                   //���Ƶ�ʳ���380MHZ
		{
			SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST ); //ѡ��PA_BOOST�ܽ�����ź�
			SX1276LoRaSetPa20dBm( true );                            //����ʿɴﵽ+20dMb
			LoRaSettings.Power = 20;                                 //���书��Ϊ20
			SX1276LoRaSetRFPower( LoRaSettings.Power );              //���÷��书��   
		}
		else
		{
			SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_RFO );
			SX1276LoRaSetPa20dBm( false );
			LoRaSettings.Power = 14;
			SX1276LoRaSetRFPower( LoRaSettings.Power );
		} 
	#endif
    SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );	                      // ���豸����Ϊ����ģʽ
}

//���ð汾��
void SX1276LoRaSetDefaults( void )
{
    SX1276Read( REG_LR_VERSION, &SX1276LR->RegVersion );
}

//SX1276��λ
//SX1276�ٷ��ṩ��ʱ��Դ����ʱ����ʹ�ö�ʱ�����õ�
void SX1276LoRaReset( void )
{
    uint32_t startTick;
	
	SX1276SetReset( RADIO_RESET_ON );
    
    startTick = GET_TICK_COUNT( );                                    //�ȴ�1ms   
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 1 ) );    

    SX1276SetReset( RADIO_RESET_OFF );
    
    startTick = GET_TICK_COUNT( );                                    //�ȴ�6ms
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 6 ) );    
}

/*****************************************************************
* ����������	����LoRa����ģʽ
* ��ڲ�����RFLR_OPMODE_SLEEP		         --	˯��ģʽ
			RFLR_OPMODE_STANDBY		         --	����ģʽ
            RFLR_OPMODE_SYNTHESIZER_TX       -- Ƶ�ʺϳɷ���ģʽ
			RFLR_OPMODE_TRANSMITTER	         --	����ģʽ
            RFLR_OPMODE_SYNTHESIZER_RX       -- Ƶ�ʺϳɽ���ģʽ
			RFLR_OPMODE_RECEIVER	         --	��������ģʽ
            RFLR_OPMODE_RECEIVER_SINGLE      -- ���ν���ģʽ
            RFLR_OPMODE_CAD                  -- �źŻ���ģʽ
* ����ֵ��		��
*******************************************************************/
void SX1276LoRaSetOpMode( uint8_t opMode )
{
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = true;
    bool antennaSwitchTxOn = false;                                  //���߿���״̬
    opModePrev = SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;
    if( opMode != opModePrev )                                       //������Ǵ���ģʽ
    {
        if( opMode == RFLR_OPMODE_TRANSMITTER )                      //����Ƿ���ģʽ
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
			RXTX( antennaSwitchTxOn );                                //�շ�ģʽ�л� RXTX����û��ʹ�� ����Ӧ��������л�
		}
		SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_MASK ) | opMode;
		//REG_LR_OPMODE�Ĵ����鿴�����ֲ�111ҳ
		SX1276Write( REG_LR_OPMODE, SX1276LR->RegOpMode );        
	}
}

//��ȡ��ǰLORA��ģʽ 
uint8_t SX1276LoRaGetOpMode( void )                       
{
    SX1276Read( REG_LR_OPMODE, &SX1276LR->RegOpMode );
    
    return SX1276LR->RegOpMode & ~RFLR_OPMODE_MASK;
}

//��ȡLNA���źŷŴ����棨001=������棬010=�������-6dB��011=�������-12dB..........��
uint8_t SX1276LoRaReadRxGain( void )                    
{
	//REG_LR_LNA�Ĵ����鿴�����ֲ�114ҳ
	SX1276Read( REG_LR_LNA, &SX1276LR->RegLna );
	return( SX1276LR->RegLna >> 5 ) & 0x07;
}

//��ȡ�ź�ǿ��
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

//��ȡ����ʱ������ֵ
uint8_t SX1276LoRaGetPacketRxGain( void )
{
    return RxGain;
}

//��ȡ����ʱ�������ֵ���źź������ı�ֵ�������Խ�ߣ�˵���źŸ���ԽС��
int8_t SX1276LoRaGetPacketSnr( void )
{
    return RxPacketSnrEstimate;
}

//��ȡ����ʱ�������ź�ǿ��
double SX1276LoRaGetPacketRssi( void )
{
    return RxPacketRssiValue;
}

//��ʼ����
void SX1276LoRaStartRx( void )
{
    SX1276LoRaSetRFState( RFLR_STATE_RX_INIT );
}

//��������
void SX1276LoRaGetRxPacket( void *buffer, uint16_t *size )
{
	*size = RxPacketSize;
	RxPacketSize = 0;
	memcpy( (void*)buffer, (void*)RFBuffer, (size_t)*size );
}

//��������
void SX1276LoRaSetTxPacket( const void *buffer, uint16_t size )
{
    if( LoRaSettings.FreqHopOn == false )    //Ĭ���ǹ��˵�
    {
        TxPacketSize = size;
    }
    else
    {
        TxPacketSize = 255;
    }
    memcpy( ( void * )RFBuffer, buffer, ( size_t )TxPacketSize );  //RFBuffer�Ǹ�װ�������ݵ����飬��buffer����Ҫ���͵����ݵ�����

    RFLRState = RFLR_STATE_TX_INIT;                                //����״̬Ϊ���ͳ�ʼ����ֻҪ������RFLR_STATE_TX_INIT���ͻῪʼ��SX1276LoRaProcess���������з�������      
}

//�õ�RFLRState״̬
uint8_t SX1276LoRaGetRFState( void )
{
    return RFLRState;
}

//����RFLRState״̬��RFLRState��ֵ����������ĺ���������һ���Ĵ���
void SX1276LoRaSetRFState( uint8_t state )
{
    RFLRState = state;
}

/*
    ���ͺͽ��յĴ�����

	����ֵ�� RF_BUSY,               //��æ״̬
             RFLR_STATE_IDLE        //����״̬
             RFLR_STATE_RX_INIT     //���ճ�ʼ��״̬
             RFLR_STATE_RX_RUNNING  //���ڽ���״̬
			 RFLR_STATE_RX_DON��    //�������
			 RFLR_STATE_RX_TIMEOUT  //���ճ�ʱ 
			 RFLR_STATE_TX_INIT     //���ͳ�ʼ��״̬
			 RFLR_STATE_TX_RUNNING  //���ڷ���״̬
			 RFLR_STATE_TX_DONE     //�������
             RFLR_STATE_CAD_RUNNIN��// 
			 RFLR_STATE_CAD_INI��
			 RFLR_STATE_TX_DONE                                    
 */
uint32_t SX1276LoRaProcess( void )
{
    uint32_t result = RF_BUSY;
    uint8_t regValue=0;
    switch( RFLRState )
    {
		case RFLR_STATE_IDLE:                                                 //����״̬������
             break;
        case RFLR_STATE_RX_INIT:                                               //������һЩ�жϣ�������Ƶ���ڣ����ö˿�ӳ�䣬���ý���ģʽ������������ʱ��
			 SX1276LoRaSetOpMode(RFLR_OPMODE_STANDBY);                         //����ģʽ
		     //REG_LR_IRQFLAGSMASK�Ĵ����������ֲ�115ҳ
		     SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT          |    
									   //RFLR_IRQFLAGS_RXDONE             |    //������������ж�
									     RFLR_IRQFLAGS_PAYLOADCRCERROR    |    //��������CRC�����ж� 
									     RFLR_IRQFLAGS_VALIDHEADER        |
									     RFLR_IRQFLAGS_TXDONE             |
									     RFLR_IRQFLAGS_CADDONE            |
                                         RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |    //����FHSS�ı��ŵ��ж�
									     RFLR_IRQFLAGS_CADDETECTED;
		     SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );    

		     if(LoRaSettings.FreqHopOn == true )                               //Ĭ��Ϊfalse
		     {
		         SX1276LR->RegHopPeriod = LoRaSettings.HopPeriod;              //Ĭ��Ϊ0   
		       //REG_LR_HOPCHANNEL�Ĵ����������ֲ�115ҳ				
                 SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );    //��ȡʹ���еĵ�Ƶ��ǰ�ŵ�ֵ 
			     SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
		     }
		     else    
		     { 
                 SX1276LR->RegHopPeriod = 255;
		     } 
		     //REG_LR_HOPPERIOD�Ĵ����������ֲ�118ҳ
		     SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );  
			
		     //�˿�ӳ������			
                                        // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
		     SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                        // CadDetected               ModeReady
		     SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_10 | RFLR_DIOMAPPING2_DIO5_00;
		
		     SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );
			
		     if( LoRaSettings.RxSingleOn == true )                               //���ν���ģʽ,Ĭ�ϳ�������ģʽ
		     {
			     SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
		     }
	         else                                                                //��������ģʽ
		     {
			     SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;
			     //REG_LR_FIFOADDRPTR�Ĵ����鿴���ݼĴ���113ҳ
			     SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );    
			     SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );
				 
		     }   
		     memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );                    //���RFBuffer
             Rx_Time_Start=TickCounter;
		     PacketTimeout = LoRaSettings.RxPacketTimeout;                       //����������ʱ��Ϊ100
		     RxTimeoutTimer = GET_TICK_COUNT( );
		     RFLRState = RFLR_STATE_RX_RUNNING;
		     break;
		case RFLR_STATE_RX_RUNNING:                                              //��ⷢ������Щ�жϣ�Ʃ�������ɣ�����������жϱ�־λ�������Ƿ���ճ�ʱ  
			 SX1276Read(0x12,&regValue);                                         //��ȡ��ʱ��������Щ�ж�  
             //if( DIO0 == 1 )                                                   //����DIO �Ͷ�ȡ�Ĵ�����־  
			 if(regValue & (1<<6))                                               //�������
             {
				 //printf("���ճɹ���\r\n");
		         RxTimeoutTimer = GET_TICK_COUNT( );
                 if( LoRaSettings.FreqHopOn == true )                             //Ĭ��false
                 { 
				     SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
					 SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
				 }
        
				 SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );           //�����־λ
				 RFLRState = RFLR_STATE_RX_DONE;
			  }
			  //if( DIO2 == 1 )                                                   //����DIO �Ͷ�ȡ�Ĵ�����־
			  if(regValue & (1<<1))                                               //FHSS�ı��ŵ�
			  {
			      RxTimeoutTimer = GET_TICK_COUNT( );
			      if( LoRaSettings.FreqHopOn == true )
				  {
					  SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
					  SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
				  }
        
				  SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL ); //�����־λ      
				  //RxGain = SX1276LoRaReadRxGain( );                               //��ȡLNA����  LNA������ǰ����߽��ܵ����źŽ��зŴ�Ϊ������׼����
			  }
			  if( LoRaSettings.RxSingleOn == true )                                 //���ν���ģʽ��Ĭ�ϳ�������ģʽ
			  {
			     if( ( GET_TICK_COUNT( ) - RxTimeoutTimer ) > PacketTimeout )       //�����Ƿ�ʱ
				 {
				      RFLRState = RFLR_STATE_RX_TIMEOUT;                            //���ճ�ʱ 
				 }
			 }
             break;
		case RFLR_STATE_RX_DONE:                                                    //�Խ������ݽ���CRC��⣬�õ���������ʱ������ȣ����Ҽ����ź�ǿ�ȣ�
			                                                                        //��ȡ�������ݵ��ֽڳ��ȣ�������SX1276��fifo�ж�ȡ���ݣ���Ϊ��ʱ
			 																		//�Է����������ݱ�SX1276���յ�����������fifo�У�
		     SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );
			 if( ( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )   //CRCУ��
			 {      
				 SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );     //�����־λ   
				 if( LoRaSettings.RxSingleOn == true )                              //���ν���ģʽ
				 {
					RFLRState = RFLR_STATE_RX_INIT;                                
				 }
				 else
				 { 
					RFLRState = RFLR_STATE_RX_RUNNING;
				 }
				 break;
		     }
		     //���´�����ÿɲ��ã���Ҫ��Ϊ�˵õ��źŵ�ǿ�Ⱥ������	
/*			 {
				 uint8_t rxSnrEstimate;
				 //REG_LR_PKTSNRVALUE�Ĵ����鿴���ݼĴ���116ҳ   
				 SX1276Read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );                     //�����յ������ݰ��� SNR Ԥ��ֵ
				 if( rxSnrEstimate & 0x80 )  //˵���Ǹ���                              // The SNR sign bit is 1
				 {

					 RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;     //����4
					 RxPacketSnrEstimate = -RxPacketSnrEstimate;
				 }
				 else
				 {
					 RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
				 }
			}        
			if( LoRaSettings.RFFrequency < 860000000 ) 
			{    
				if( RxPacketSnrEstimate < 0 )          //SNR�������С��0ʱ
				{
					//���������ź�ǿ�ȣ������Ǽ��㹫ʽ����Ƶ�͵�Ƶ�ź��е㲻ͬ��SNRֵ������Ҳ�᲻ͬ
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
			if( LoRaSettings.RxSingleOn == true )                                     //���ν���ģʽ��Ĭ�ϳ���ģʽ
			{
			    SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;       
				SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );          //��ȡ��������
			    if( LoRaSettings.ImplicitHeaderOn == true )                           //Ĭ��false
				{
				    RxPacketSize = SX1276LR->RegPayloadLength;
				    SX1276ReadFifo( RFBuffer, SX1276LR->RegPayloadLength );           //��Lora�ж�ȡ���յ�������
				}
				else
				{
					//REG_LR_NBRXBYTES�Ĵ����鿴���ݼĴ���115ҳ  ���һ�ν��յ������ݰ��ĸ����ֽ���
				    SX1276Read( REG_LR_NBRXBYTES, &SX1276LR->RegNbRxBytes );
					RxPacketSize = SX1276LR->RegNbRxBytes;
				    SX1276ReadFifo( RFBuffer, SX1276LR->RegNbRxBytes );
			     }
			 }
			 else                                                                     //��������ģʽ
			 {
			     //REG_LR_FIFORXCURRENTADDR�Ĵ����鿴���ݼĴ���115ҳ   ���յ����һ�����ݰ�����ʼ��ַ�����ݻ������У�
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
			 if( LoRaSettings.RxSingleOn == true )                                     //���ν���ģʽ
			 { 
				 RFLRState = RFLR_STATE_RX_INIT;
			 }
			 else                                                                      //��������ģʽ
			 { 
				 RFLRState = RFLR_STATE_RX_RUNNING;
			 }
			 Rx_Time_End=TickCounter;
			 result = RF_RX_DONE;
			 break;
		case RFLR_STATE_RX_TIMEOUT:                                                   //����ʱ�䳬ʱ
		     RFLRState = RFLR_STATE_RX_INIT;                                          //�����¿�ʼ���գ��ӳ�ʼ����ʼ
		     result = RF_RX_TIMEOUT;
		     break;
		case RFLR_STATE_TX_INIT:                                                      //���ͳ�ʼ����������������жϣ�
																					  //���õ�Ƶ����Ϊ0�����÷������ݵĳ��ȣ�����Ϊ����ģʽ
		     Tx_Time_Start=TickCounter;
			 SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );                              //����ģʽ
			 if( LoRaSettings.FreqHopOn == true )                                     //��Ƶ,Ĭ��false
			 {
				 SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT          |
											 RFLR_IRQFLAGS_RXDONE             |
											 RFLR_IRQFLAGS_PAYLOADCRCERROR    |
											 RFLR_IRQFLAGS_VALIDHEADER        |
										   //RFLR_IRQFLAGS_TXDONE             |       //������������ж�
											 RFLR_IRQFLAGS_CADDONE            |
										   //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |       //����FHSS�ı��ŵ��ж�
											 RFLR_IRQFLAGS_CADDETECTED;
				 SX1276LR->RegHopPeriod = LoRaSettings.HopPeriod;
				 SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
				 SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
			 }
			 else
			 {
				 //REG_LR_IRQFLAGSMASK�Ĵ����������ֲ�115
				 SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT          |
									         RFLR_IRQFLAGS_RXDONE             |
											 RFLR_IRQFLAGS_PAYLOADCRCERROR    |
											 RFLR_IRQFLAGS_VALIDHEADER        |
										   //RFLR_IRQFLAGS_TXDONE             |      //������������ж�
											 RFLR_IRQFLAGS_CADDONE            |
											 RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
											 RFLR_IRQFLAGS_CADDETECTED;             
				 SX1276LR->RegHopPeriod = 0;
			 }
			 SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );                //������Ƶ���ڣ�Ĭ��Ϊ0
			 SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );      

			 SX1276LR->RegPayloadLength = TxPacketSize;                              //��ʼ�����ش�С
			 SX1276Write( REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength );
			
			 SX1276LR->RegFifoTxBaseAddr = 0x00;                                     
			 SX1276Write( REG_LR_FIFOTXBASEADDR, SX1276LR->RegFifoTxBaseAddr );

			 SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoTxBaseAddr;
			 SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );
			  
			 SX1276WriteFifo( RFBuffer, SX1276LR->RegPayloadLength );                //��Ҫ���͵�����д��LORA��fifo��
			 								// TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
			 SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
											// PllLock              Mode Ready
			 SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
			
			 SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );

			 SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );

			 RFLRState = RFLR_STATE_TX_RUNNING;
			 break;
		case RFLR_STATE_TX_RUNNING:                                            //����״̬����ⷢ����Щ�жϣ�Ʃ�緢������жϣ�
			 SX1276Read(0x12,&regValue);
		     //if( DIO0 == 1 )                                                 //�������
			 if(regValue & (1<<3))
			 {
		         printf("���ͳɹ���\r\n");
				 SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );          //�����־λ
				 RFLRState = RFLR_STATE_TX_DONE;   
			 }	
			 //if( DIO2 == 1 )                                                  //FHSS�ı��ŵ�
			 if(regValue & (1<<3))
			 {
				 if( LoRaSettings.FreqHopOn == true )
				 {
					 SX1276Read( REG_LR_HOPCHANNEL, &SX1276LR->RegHopChannel );
					 SX1276LoRaSetRFFrequency( HoppingFrequencies[SX1276LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
				 }				
				 SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL ); //�����־λ
			 }
			 break;
		case RFLR_STATE_TX_DONE:                                                //������� �ֻص�����״̬ 
             Tx_Time_End=TickCounter;			
			 SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );                         //ͨ�������ݰ����ͺ������رշ�������Ż�����
			 RFLRState = RFLR_STATE_IDLE;
			 result = RF_TX_DONE;
			 break;
		case RFLR_STATE_CAD_INIT:                                               //CAD�źŻ���
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
//��Ϊ����Ĵ�����ֱ��һ��κܸ��ӣ����������Ҽ����£���ʵ���Ƿֳ��˼�������
/*******************************************************************************/

/********************************************************************
	* ����������	��ȡ���ñ�����ǰ���ӵ��ǰ����ݵ������
	* ��ڲ�����	��
	* ����ֵ��		����ȣ������и�
*********************************************************************/
int8_t GetLoRaSNR (void)                                                //512��526�����ⲿ�ִ���
{
	u8 rxSnrEstimate;
	//ȡ���һ�����ݵ������         									//��¼����ȹ�ֵ
	SX1276Read(REG_LR_PKTSNRVALUE, &rxSnrEstimate );					//�����
	//The SNR sign bit is 1												//�����С��0�����������
	if( rxSnrEstimate & 0x80 )
	{
		RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;	//Invert and divide by 4
		RxPacketSnrEstimate = -RxPacketSnrEstimate;
	}
	else
	{
		//����ȴ���0�����������
		RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;			//Divide by 4
	}
	return RxPacketSnrEstimate;
}

//�õ��ź����棬��LNA
uint8_t GetLoRaLNA()
{
    return SX1276LoRaReadRxGain();
}

/*******************************************************************************
	* ��������:     ��ȡ���ñ�����ǰ���ӵ��ǰ����ݵ��ź�ǿ��
	* ��ڲ���:     ��
	* ����ֵ:       �ź�ǿ�ȣ������и�
********************************************************************************/
double GetPackLoRaRSSI(void)                                             //527�е�551��
{
	if( GetLoRaSNR() < 0 )
	{
		/***************************************************************************************
			*���ʣ�P=-174��dBm�� + BW(dB) + NF(dB) + SNR(dB);
			*���źű�������û��������ô˹�ʽ����RSSI��ǰ����������������,���һ���������
		****************************************************************************************/
		//�źű�������û
		RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + \
		                    NOISE_FIGURE_LF     + (double)RxPacketSnrEstimate;
	}
	else
	{
		//�ź�ǿ������
		SX1276Read( REG_LR_PKTRSSIVALUE, &SX1276LR->RegPktRssiValue );
		RxPacketRssiValue = RssiOffsetLF[LoRaSettings.SignalBw] + (double)SX1276LR->RegPktRssiValue;
	}
	return RxPacketRssiValue;
}

/*********************************************************************************************************
	* ��������:      ��ȡ��ǰ�ź�ǿ��,���ص��ǵ������������һ�̵��ź�ǿ����Ҫ�ڽ���״̬�µ��ò����á�
	* ��ڲ���:      ��
	* ����ֵ:        ��
*********************************************************************************************************/
//double SX1276LoRaReadRssi( void )�⺯�������ж��壬ȥ���濴����


/*********************************************************************************************************
	* ��������:      �������״̬
	* ��ڲ���:      ��
	* ����ֵ:        ��
*********************************************************************************************************/
void LoRaRxStateEnter (void)
{
	//1.---->�������ģʽ
	SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );		//�����Ĵ���ֻ������Standby,sleep or FSTX ģʽ

	//2.---->���ճ�ʼ������������
	SX1276LR->RegIrqFlagsMask	= 	RFLR_IRQFLAGS_RXTIMEOUT          |	//��ʱ�ж�����
	                              //RFLR_IRQFLAGS_RXDONE             |	//�����ݰ���������ж�
	                              //RFLR_IRQFLAGS_PAYLOADCRCERROR    |	//����CRC�����ж�����
	                                RFLR_IRQFLAGS_VALIDHEADER        |	//Rxģʽ�½��յ�����Ч��ͷ����
	                                RFLR_IRQFLAGS_TXDONE             |	//FIFO���ط�������ж�����
	                                RFLR_IRQFLAGS_CADDONE            |	//CAD����ж�����
	                                RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |	//FHSS�ı��ŵ��ж�����
	                                RFLR_IRQFLAGS_CADDETECTED;			//��⵽CAD�ж�����
	SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );		//������Ҫ���ε��ж�(��ע�͵����жϼ���)

	SX1276LR->RegHopPeriod = 255;
	SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );			//Ƶ������֮��ķ�������:255

	//DIO0:RxDone
	SX1276LR->RegDioMapping1 = 	RFLR_DIOMAPPING1_DIO0_00 |				//DIO0~DIO3����ӳ��
	                            RFLR_DIOMAPPING1_DIO1_11 |
	                            RFLR_DIOMAPPING1_DIO2_11 |
	                            RFLR_DIOMAPPING1_DIO3_11;

	SX1276LR->RegDioMapping2 = 	RFLR_DIOMAPPING2_DIO4_10 | 				//DIO4~DIO5����ӳ��
	                            RFLR_DIOMAPPING2_DIO5_11;

	/*****************************************************************************
		*��ZM470SX-M��,DIO4�ӵ���Ƶ���ص�6��,�������״̬���������Ҫ����,
		*������LoRaģʽ������Ϊ:RFLR_DIOMAPPING2_DIO4_10
	******************************************************************************/
	SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );	//����5��IO�Ĺ���

	//3.---->���ý���ģʽ(����/����ģʽ)
	if( LoRaSettings.RxSingleOn == true )
	{
		//����Ϊ���ν���
		SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
	}
	else
	{
		//����Ϊ��������ģʽ
		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxBaseAddr;		  	//����:0x00
		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );	//SPI����FIFO�ĵ�ַ
		SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );           			//�������״̬
	}
//	PacketTimeout  = LoRaSettings.RxPacketTimeout;						//��ʱʱ��Ϊ 100
//	RxTimeoutTimer = GET_TICK_COUNT( );									//���ճ�ʱʱ��Ϊ 10
}

/**************************************************************
	* ����������   ��ȡ���յ�����
	* ��ڲ�����	pbuf	--	���ݻ�����ָ��
					size	--	�����ֽڳ���ָ��
	* ����ֵ��		��
***************************************************************/
void LoRaRxDataRead (u8 *pbuf, u8 *size )
{
	if( DIO0 == 1 ) 
	{                                                  				//RxDone  �������                    
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );		//ִ��д��������������жϱ�־
	}	
	
    SX1276Read( REG_LR_IRQFLAGS, &SX1276LR->RegIrqFlags );			//��ȡ�ж�״̬��־λ    
	//���CRC��־������֤���ݰ��������� 	
	if( ( SX1276LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
	{
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );	//ִ��д��������������жϱ�־
		*size = 0;
		return;
	}     
	
//	GetLoRaSNR();                  //��ȡ���һ�����������(�����������SNR�����ɰѴ˺���ɾ��)     	     
//	GetPackLoRaRSSI();             //��ȡ���һ�������ź�ǿ��(�����������RSSI�����ɰѴ˺���ɾ��)

	/************* ȡ���� ************/
	SX1276Read( REG_LR_FIFORXCURRENTADDR, &SX1276LR->RegFifoRxCurrentAddr );  	//��ȡ������һ��������ָ��
	if( LoRaSettings.ImplicitHeaderOn == true ) 
	{
		RxPacketSize = SX1276LR->RegPayloadLength;
		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;
		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );			//д��FIFO�ķ��ʵ�ַ
		SX1276ReadFifo( RFBuffer, SX1276LR->RegPayloadLength );
	} 
	//��ʾ��ͷģʽ
	else 
	{
		SX1276Read( REG_LR_NBRXBYTES, size );	 						//��ȡ������(size��¼���ݰ��ĳ���)
//		RxPacketSize = SX1276LR->RegNbRxBytes;							//�õ����հ��Ĵ�С
		SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoRxCurrentAddr;		//�������յ����ݰ����׵�ַ����FIFO���ݻ������ķ��ʵ�ַ(SPI)
		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );  	//д��FIFO�ķ��ʵ�ַ
		SX1276ReadFifo( pbuf, *size);									//��ȡFIFO�е����ݣ���������pbuf��
	}
}	

u16 Tx_Counter = 0;
/****************************************************************
	* ����������	��������
	* ��ڲ�����	pbuf		--	���ݻ�����ָ��
					size		--	�����ֽ���
	* ����ֵ��		0			--	���ͳɹ�
					����(1,2)	--	����ʧ��
******************************************************************/
u8 LoRaTxData (u8 *pbuf, u8 size ,u8 *pcrcbuf ,u8 crcflag)
{
     u32 i,j; 
	if (0 == pbuf || 0 == size)	//û�����ݿɷ���
	{
		return 1;				//����ʧ�� -- ����1
	}		 
	
    SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );

	SX1276LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT          |
								RFLR_IRQFLAGS_RXDONE             |
								RFLR_IRQFLAGS_PAYLOADCRCERROR    |
								RFLR_IRQFLAGS_VALIDHEADER        |
								//RFLR_IRQFLAGS_TXDONE           |		//�򿪷�������ж�(Tx)
								RFLR_IRQFLAGS_CADDONE            |
								RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
								RFLR_IRQFLAGS_CADDETECTED;
	SX1276LR->RegHopPeriod = 0;											//FHSS��Ƶ����Ϊ0
	SX1276Write( REG_LR_HOPPERIOD, SX1276LR->RegHopPeriod );
	SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR->RegIrqFlagsMask );		//д���жϱ�־λ

	//Initializes the payload size
	SX1276LR->RegPayloadLength = size; 									//���ݸ��س���(������CRC����)
	SX1276Write( REG_LR_PAYLOADLENGTH, SX1276LR->RegPayloadLength +4); 	//��implicitģʽ(��ʽ��ͷ),����д��FIFO����,0x80

	SX1276LR->RegFifoTxBaseAddr = 0x00;                                 //Full buffer used for Tx     
	SX1276Write( REG_LR_FIFOTXBASEADDR, SX1276LR->RegFifoTxBaseAddr );  //д�뷢�͵��׵�ַ 

	SX1276LR->RegFifoAddrPtr = SX1276LR->RegFifoTxBaseAddr;
	SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );        //д��FIFO�ķ��ʵ�ַ
	SX1276WriteFifo( pbuf, SX1276LR->RegPayloadLength );	            //д������Ҫ���͵�����
	//SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR->RegFifoAddrPtr );      //д��FIFO�ķ��ʵ�ַ
	
	//д��CRCУ��ֵ
	if(crcflag==1)
	{
		if( size<(256-4) )
			SX1276WriteFifo( pcrcbuf, 4 );
		else
			return 1;
	}
	
	/*****************************************************************************
		*��ZM470SX-M��,DIO4�ӵ���Ƶ���ص�6��,���뷢��״̬���������Ҫ����,
		*������LoRaģʽ������Ϊ:RFLR_DIOMAPPING2_DIO4_00
	******************************************************************************/
    //DIO0:TxDone  �˿�ӳ������ 
	SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_11| 
	                           RFLR_DIOMAPPING1_DIO2_11 | RFLR_DIOMAPPING1_DIO3_11;
	SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_11;
	SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );
	///////////////////////////////////////////////////////////////////////////////
    SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );		//���뷢��״̬ 
	
	
    printf("��ʼ����ʱ��Ϊ		--	t1:%d\r\n",TickCounter);
	i = 80000; 
	while ( DIO0 == 0 && i != 0)		//�ȴ�������ɻ��߷���ʧ�ܳ�ʱ,��ɺ�:DIO0Ϊ�ߵ�ƽ
	{                 
        i--;	
		for(j = 0; j < 100; j++);		//��ʱ�ȴ�Լ1ms        
	}
	
	printf("i��ֵΪ -- %d��\r\n",80000-i);
	//�������ʧ��   
	if (i == 0) 
	{   		
		printf("����ʧ�� -- 2\r\n");
		return 2;	//ʧ�� -- ����2
	} 
	//������ͳɹ�
	else 
	{					               
		SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );	//�����������ж�           
		SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );				//�������ģʽ
		printf("�ɹ��������ʱ��Ϊ	--	t2:%d\r\n",TickCounter);	
		printf("���ͳɹ� -- 0\r\n");
		return 0;												//�ɹ� -- ����0
	}
	//SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );			//���뷢��״̬            
}

/******************************************************
	* ����������    ���÷��书��
	* ��ڲ�����    pwr: ���ʷ�Χ5 ~ 20
	* ����ֵ��      ��
*******************************************************/
void LoRaTxPower (u8 pwr)
{
	//Ĭ�ϲ���PA���ڵ���Ϊ100mA,�����20dBmʱ��Ҫ120mA,���Ա�������0x0b����
	SX1276Write( REG_LR_OCP, 0x3f );
	SX1276LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST );
	SX1276LoRaSetPa20dBm( true );
	SX1276LoRaSetRFPower( pwr);
}

/******************************************************
	* ����������	�����ز�Ƶ��
	* ��ڲ�����    420000000~500000000
	* ����ֵ��      ��
*******************************************************/
void LoRaFreqSet (u32 freq )
{
	SX1276LoRaSetRFFrequency( freq );
}
#endif 

