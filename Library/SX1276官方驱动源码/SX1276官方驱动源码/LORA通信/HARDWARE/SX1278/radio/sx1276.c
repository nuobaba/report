#include "platform.h"
#include "radio.h"
#include "usart.h"

#if defined( USE_SX1276_RADIO )
#include "sx1276.h"
#include "sx1276-Hal.h"
#include "sx1276-Fsk.h"
#include "sx1276-LoRa.h"
#include "delay.h"

uint8_t SX1276Regs[0x70];        //SX1276寄存器数组

static bool LoRaOn = true;       //false代表LORA模式关闭 
static bool LoRaOnState = false; //false代表LORA模式关闭 

void SX1276_Init( void )
{
	uint8_t TempReg;
	
    SX1276=(tSX1276*)SX1276Regs;    //初始化FSK寄存器结构
    SX1276LR=(tSX1276LR*)SX1276Regs;//初始化LoRa寄存器结构

    SX1276_Init_IO( );              //SX1276的IO口初始化         
    SX1276_Reset( );                 //重置SX1276
	
	SX1276Read(0x06,&TempReg);      //用于测试spi是否能读写数据
	while(TempReg != 0x6C)
	{
		printf("Hard SPI Err!\r\n");
		delay_ms(100);
	}
    //注:无线电复位后，默认调制解调器为FSK
    #if ( LORA == 0 )               //LORA = 0
		LoRaOn = false;
		SX1276_SetLoRaOn( LoRaOn );
		SX1276FskInit( );//初始化FSK模式
	#else
		LoRaOn = true;               //LORA = 1
		SX1276_SetLoRaOn( LoRaOn );
		SX1276LoRaInit( );       //初始化LoRa模式   
	#endif
}

//重置SX1276
//SX1276官方提供是时钟源即延时函数使用定时器设置的
void SX1276_Reset( void )
{
	uint32_t startTick;  

	SX1276SetReset( RADIO_RESET_ON );
    
    startTick = GET_TICK_COUNT( );   //等待1ms
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 1 ) );    

    SX1276SetReset( RADIO_RESET_OFF );
    
    startTick = GET_TICK_COUNT( );   //等待6ms
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 6 ) );    
}

//启用LoRa模式
void SX1276_SetLoRaOn( bool enable )
{
    if( LoRaOnState == enable )
    {
        return;
    }
    LoRaOnState = enable;
    LoRaOn = enable;

    if( LoRaOn == true )
    {
        SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );   //进入睡眠模式,才可以修改OPMODE
		
        /*RegOpMode这个寄存器（0x01）最高位用来设置LORA还是FSK，1位LORA，0位FSK*/
        SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON;
        SX1276Write( REG_LR_OPMODE, SX1276LR->RegOpMode );
        
        SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );  //设置为待机模式
		
		/*REG_LR_DIOMAPPING1（0x40）这个寄存器用来设置LORA模式下DIO0-DIO5的中断映射关系*/
                                        // RxDone               RxTimeout                   FhssChangeChannel           CadDone
        SX1276LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                        // CadDetected          ModeReady
        SX1276LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR->RegDioMapping1, 2 );
        
        SX1276ReadBuffer( REG_LR_OPMODE, SX1276Regs + 1, 0x70 - 1 );  //将REG_LR_OPMODE寄存器值保存在SX1276Regs[0x70-1]寄存器组中
    }
    else      //否则设置为FSK模式
    {
        SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
        
        SX1276LR->RegOpMode = ( SX1276LR->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF;
        SX1276Write( REG_LR_OPMODE, SX1276LR->RegOpMode );
        
        SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
        
        SX1276ReadBuffer( REG_OPMODE, SX1276Regs + 1, 0x70 - 1 );
    }
}

//获取LoRa调制解调器状态
bool SX1276_GetLoRaOn( void )
{
    return LoRaOn;
}

//设置SX1276操作模式
void SX1276SetOpMode( uint8_t opMode )
{
    if( LoRaOn == false )
    {
        SX1276FskSetOpMode( opMode );
    }
    else
    {
        SX1276LoRaSetOpMode( opMode );
    }
}

//获取SX1276操作模式
uint8_t SX1276_GetOpMode( void )
{
    if( LoRaOn == false )
    {
        return SX1276FskGetOpMode( );
    }
    else
    {
        return SX1276LoRaGetOpMode( );
    }
}

//读取无线信号强度
double SX1276ReadRssi( void )
{
    if( LoRaOn == false )
    {
        return SX1276FskReadRssi( );
    }
    else
    {
        return SX1276LoRaReadRssi( );
    }
}

//读取当前Rx增益设置
uint8_t SX1276_ReadRxGain( void )
{
    if( LoRaOn == false )
    {
        return SX1276FskReadRxGain( );
    }
    else
    {
        return SX1276LoRaReadRxGain( );
    }
}

//获取数据时的增益值
uint8_t SX1276_GetPacketRxGain( void )
{
    if( LoRaOn == false )
    {
        return SX1276FskGetPacketRxGain(  );
    }
    else
    {
        return SX1276LoRaGetPacketRxGain(  );
    }
}

//获取数据时的信噪比值，信号和噪声的比值，信噪比越高，说明信号干扰越小。
int8_t SX1276_GetPacketSnr( void )
{
    if( LoRaOn == false )
    {
         while( 1 )
         {
             // Useless in FSK mode
             // Block program here
         }
    }
    else
    {
        return SX1276LoRaGetPacketSnr(  );
    }
}

//获取数据是的无线信号强度
double SX1276_GetPacketRssi( void )
{
    if( LoRaOn == false )
    {
        return SX1276FskGetPacketRssi(  );
    }
    else
    {
        return SX1276LoRaGetPacketRssi( );
    }
}

uint32_t SX1276GetPacketAfc( void )
{
    if( LoRaOn == false )
    {
        return SX1276FskGetPacketAfc(  );
    }
    else
    {
         while( 1 )
         {
             // Useless in LoRa mode
             // Block program here
         }
    }
}

//开始接收
void SX1276StartRx( void )
{
  if( LoRaOn == false )
  {
      SX1276FskSetRFState( RF_STATE_RX_INIT );
  }
  else
  {
      SX1276LoRaSetRFState( RFLR_STATE_RX_INIT );    //LoRa 中断接收状态
  }
}

//得到接收的数据
void SX1276GetRxPacket( void *buffer, uint16_t *size )
{
    if( LoRaOn == false )
    {
        SX1276FskGetRxPacket( buffer, size );
    }
    else
    {
        SX1276LoRaGetRxPacket( buffer, size );
    }
}

//发送数据
void SX1276SetTxPacket( const void *buffer, uint16_t size )
{
    if( LoRaOn == false )
    {
        SX1276FskSetTxPacket( buffer, size );
    }
    else
    {
        SX1276LoRaSetTxPacket( buffer, size );
    }
}

//得到RFLRState状态
uint8_t SX1276GetRFState( void )
{
    if( LoRaOn == false )
    {
        return SX1276FskGetRFState( );
    }
    else
    {
        return SX1276LoRaGetRFState( );
    }
}

//设置RFLRState状态，RFLRState的值决定了下面的函数处理哪一步的代码
void SX1276SetRFState( uint8_t state )
{
    if( LoRaOn == false )
    {
        SX1276FskSetRFState( state );
    }
    else
    {
        SX1276LoRaSetRFState( state );
    }
}

//SX1276模块接发收数据的处理函数
uint32_t SX1276Process( void )
{
    if( LoRaOn == false )
    {
        return SX1276FskProcess( );
    }
    else
    {
        return SX1276LoRaProcess( );
    }
}




////////////////////////////////////
/*以下代码也是根据自己理解写的代码*/
////////////////////////////////////

//读取包信噪比
int8_t SX1276GetPacketSnr( void )
{
	return GetLoRaSNR ();
}


//读取包信号强度
double SX1276GetPacketRssi( void )
{
	return GetPackLoRaRSSI();
}


//读取当前信号强度
/*double SX1276ReadRssi( void )
{
	return SX1276LoRaReadRssi( );
}*/


//进入接收状态
void SX1276RxStateEnter( void )
{
	LoRaRxStateEnter ();
}


//读取接收到的数据 
void SX1276RxDataRead(u8 *pbuf, u8 *size )
{
	LoRaRxDataRead (pbuf,size );
}


//发送数据
u8 SX1276TxData(u8 *pbuf, u8 size )
{
	return LoRaTxData(pbuf, size,"000",0);
}


//设置发射功率
void SX1276TxPower(u8 pwr )
{
	LoRaTxPower (pwr);
}


//设置频率载波 
void SX1276FreqSet(u32 freq)
{
	LoRaFreqSet(freq);
}

/* CRC32数值表 */
static const u32 crc32tab[] = {
 0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL,
 0x076dc419L, 0x706af48fL, 0xe963a535L, 0x9e6495a3L,
 0x0edb8832L, 0x79dcb8a4L, 0xe0d5e91eL, 0x97d2d988L,
 0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L, 0x90bf1d91L,
 0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
 0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L,
 0x136c9856L, 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL,
 0x14015c4fL, 0x63066cd9L, 0xfa0f3d63L, 0x8d080df5L,
 0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L, 0xa2677172L,
 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
 0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L,
 0x32d86ce3L, 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L,
 0x26d930acL, 0x51de003aL, 0xc8d75180L, 0xbfd06116L,
 0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L, 0xb8bda50fL,
 0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
 0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL,
 0x76dc4190L, 0x01db7106L, 0x98d220bcL, 0xefd5102aL,
 0x71b18589L, 0x06b6b51fL, 0x9fbfe4a5L, 0xe8b8d433L,
 0x7807c9a2L, 0x0f00f934L, 0x9609a88eL, 0xe10e9818L,
 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
 0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL,
 0x6c0695edL, 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L,
 0x65b0d9c6L, 0x12b7e950L, 0x8bbeb8eaL, 0xfcb9887cL,
 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L, 0xfbd44c65L,
 0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
 0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL,
 0x4369e96aL, 0x346ed9fcL, 0xad678846L, 0xda60b8d0L,
 0x44042d73L, 0x33031de5L, 0xaa0a4c5fL, 0xdd0d7cc9L,
 0x5005713cL, 0x270241aaL, 0xbe0b1010L, 0xc90c2086L,
 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
 0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L,
 0x59b33d17L, 0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL,
 0xedb88320L, 0x9abfb3b6L, 0x03b6e20cL, 0x74b1d29aL,
 0xead54739L, 0x9dd277afL, 0x04db2615L, 0x73dc1683L,
 0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
 0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L,
 0xf00f9344L, 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL,
 0xf762575dL, 0x806567cbL, 0x196c3671L, 0x6e6b06e7L,
 0xfed41b76L, 0x89d32be0L, 0x10da7a5aL, 0x67dd4accL,
 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
 0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L,
 0xd1bb67f1L, 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL,
 0xd80d2bdaL, 0xaf0a1b4cL, 0x36034af6L, 0x41047a60L,
 0xdf60efc3L, 0xa867df55L, 0x316e8eefL, 0x4669be79L,
 0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
 0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL,
 0xc5ba3bbeL, 0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L,
 0xc2d7ffa7L, 0xb5d0cf31L, 0x2cd99e8bL, 0x5bdeae1dL,
 0x9b64c2b0L, 0xec63f226L, 0x756aa39cL, 0x026d930aL,
 0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
 0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L,
 0x92d28e9bL, 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L,
 0x86d3d2d4L, 0xf1d4e242L, 0x68ddb3f8L, 0x1fda836eL,
 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L, 0x18b74777L,
 0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
 0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L,
 0xa00ae278L, 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L,
 0xa7672661L, 0xd06016f7L, 0x4969474dL, 0x3e6e77dbL,
 0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L, 0x37d83bf0L,
 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
 0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L,
 0xbad03605L, 0xcdd70693L, 0x54de5729L, 0x23d967bfL,
 0xb3667a2eL, 0xc4614ab8L, 0x5d681b02L, 0x2a6f2b94L,
 0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL, 0x2d02ef8dL
};


//保存CRC32校验值
unsigned char crc32V[4] = {0};


//生成CRC校验值
void crc32( const unsigned char *buf, u32 size)
{
	u32 i, crc;
	crc = 0xFFFFFFFF;
	for (i=0; i<size; i++)
		crc = crc32tab[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
	crc ^= 0xFFFFFFFF;		//按位异或值
	
	crc32V[0] = crc>>24;	//31 ~ 24
	crc32V[1] = crc>>16;	//16 ~ 23
	crc32V[2] = crc>>8;		// 8 ~ 15
	crc32V[3] = crc;		// 0 ~ 7
}

//CRC32校验的发送函数，输入参数size最大为256-4=252(CRC校验值占4Byte) 
u8 SX1276TxDataCRC32(u8 *pbuf, u8 size )
{
	crc32(pbuf,size);						//生成CRC校验值
	return LoRaTxData(pbuf, size,crc32V,1);	//发送数据包(...+数据+CRC校验值)
}

//CRC32校验的接收函数，输入参数pbuf应该要比可能接收到的数据包的最大长度长4字节
u8 SX1276RxDataReadCRC32(u8 *pbuf, u8 *size )
{
	u8 Rxsize=0;
	LoRaRxDataRead( pbuf,&Rxsize );
	/**************add by zwq at 2019-3-22**************/
	//-----------------------------------------------------
	if( Rxsize<4 )
		return 2;
	
	crc32(pbuf,Rxsize-4);
	
	if( Rxsize>240 )
		return 3;
	//-----------------------------------------------------
	crc32(pbuf,Rxsize-4);
	if(	(pbuf[Rxsize-4] == crc32V[0]) && 
		(pbuf[Rxsize-3] == crc32V[1]) && 
		(pbuf[Rxsize-2] == crc32V[2]) && 
		(pbuf[Rxsize-1] == crc32V[3])   )
	{
		*size = Rxsize-4;
		return 0;
	}
	*size = 0;
	return 1;
}
/********************************************The end**************************************************/
#endif 
