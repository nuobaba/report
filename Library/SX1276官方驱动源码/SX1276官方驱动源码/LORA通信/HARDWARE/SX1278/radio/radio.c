#include <stdint.h>
#include "platform.h"
#include "radio.h"

#if defined( USE_SX1232_RADIO )
    #include "sx1232.h"
#elif defined( USE_SX1272_RADIO )
    #include "sx1272.h"
#elif defined( USE_SX1276_RADIO )
    #include "sx1276.h"
#else
    #error "Missing define: USE_XXXXXX_RADIO (ie. USE_SX1272_RADIO)"
#endif    

tRadioDriver RadioDriver;

tRadioDriver* RadioDriverInit( void )
{
#if defined( USE_SX1232_RADIO )
    RadioDriver.Init = SX1232Init;
    RadioDriver.Reset = SX1232Reset;
    RadioDriver.StartRx = SX1232StartRx;
    RadioDriver.GetRxPacket = SX1232GetRxPacket;
    RadioDriver.SetTxPacket = SX1232SetTxPacket;
    RadioDriver.Process = SX1232Process;
#elif defined( USE_SX1272_RADIO )
    RadioDriver.Init = SX1272Init;
    RadioDriver.Reset = SX1272Reset;
    RadioDriver.StartRx = SX1272StartRx;
    RadioDriver.GetRxPacket = SX1272GetRxPacket;
    RadioDriver.SetTxPacket = SX1272SetTxPacket;
    RadioDriver.Process = SX1272Process;
#elif defined( USE_SX1276_RADIO )
    RadioDriver.Init = SX1276_Init;
    RadioDriver.Reset = SX1276_Reset;
    RadioDriver.StartRx = SX1276StartRx;
    RadioDriver.GetRxPacket = SX1276GetRxPacket;
    RadioDriver.SetTxPacket = SX1276SetTxPacket;
    RadioDriver.Process = SX1276Process;
#else
    #error "Missing define: USE_XXXXXX_RADIO (ie. USE_SX1272_RADIO)"
#endif    

    return &RadioDriver;
}
