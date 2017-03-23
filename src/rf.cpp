
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

#include "fifo.h"
#include "rf.h"
#include "rfm95.h"
#include "spi.h"
#include "uart1.h"

inline void RFM95_RESET_On(void) {
    GPIO_SetBits(GPIOB, GPIO_Pin_5);
}
inline void RFM95_RESET_Off(void) {
    GPIO_ResetBits(GPIOB, GPIO_Pin_5);
}

static void RFM95_RESET(uint8_t On) {
    if (On)
        RFM95_RESET_On();
    else
        RFM95_RESET_Off();
}

void RFM95_GPIO_Configuration(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB /* | RCC_APB2Periph_AFIO */, ENABLE);

    //    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;  // PB4 = DIO0 and PB3 = DIO4 of RFM69
    //    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    //    // GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    //    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // #TODO init DIO0-5

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;  // PB5 = RESET (active high)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// OGN frequencies: 868.2 and 868.4 MHz
static const double OGN_BaseFreq = 868.200e6;  // [MHz] 868.2MHz base frequency
static const double OGN_ChanSpace = 0.200e6;   // [MHz]   0.2MHz channel spacing
static const double XtalFreq = 32e6;           // [MHz] RF chip crystal frequency

static const uint32_t BaseFreq = (OGN_BaseFreq / (XtalFreq / (1 << 19)) + 0.5);    // conversion from RF frequency
static const uint32_t ChanSpace = (OGN_ChanSpace / (XtalFreq / (1 << 19)) + 0.5);  // to RF chip synthesizer setting

static const uint8_t OGN_SYNC[8] = {0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A};

static RFM95 TRX;

static uint8_t RxPktData[26];  // received packet data
static uint8_t RxPktErr[26];   // received packet error pattern

static uint8_t Transmit(const uint8_t *PacketByte, uint8_t Thresh, uint8_t MaxWait = 7) {
    if (PacketByte == 0) return 0;  // if no data: simply return

    //    for (; MaxWait; MaxWait--)  // wait for a given maximum time for a free radio channel
    //    {
    //        TRX.TriggerRSSI();
    //        vTaskDelay(1);
    //        uint8_t RxRSSI = TRX.ReadRSSI();
    //        RX_Random = (RX_Random << 1) | (RxRSSI & 1);
    //        if (RxRSSI >= Thresh) break;
    //    }
    //    if (MaxWait == 0) return 0;

    TRX.WriteMode(RH_RF95_MODE_STDBY);  // switch to standby
    // vTaskDelay(1);

    // TRX.WriteTxPower(Parameters.getTxPower(), Parameters.isTxTypeHW());  // set TX for transmission
    TRX.WriteSYNC(8, 7, OGN_SYNC);  // Full SYNC for TX
    TRX.ClearIrqFlags();
    TRX.WritePacket(PacketByte);           // write packet into FIFO
    TRX.WriteMode(RH_RF95_MODE_TX);        // transmit
    for (uint8_t Wait = 10; Wait; Wait--)  // wait for transmission to end
    {
        // vTaskDelay(1);
        uint8_t Mode = TRX.ReadMode();
        uint16_t Flags = TRX.ReadIrqFlags();
        if (Mode != RH_RF95_MODE_TX) break;
        if (Flags & RH_RF95_IRQ_PacketSent) break;
    }
    TRX.WriteMode(RH_RF95_MODE_STDBY);  // switch to standby

    TRX.WriteTxPowerMin();           // setup for RX
    TRX.WriteSYNC(7, 7, OGN_SYNC);   // write (not complete) SYNC
    TRX.WriteMode(RH_RF95_MODE_RX);  // back to receive mode

    return 1;
}

void StartRFchip(void) {
    RFM95_GPIO_Configuration();
    TRX.Select = SPI1_Select;  //
    TRX.Deselect = SPI1_Deselect;
    TRX.TransferByte = SPI1_TransferByte;
    TRX.RESET_On = RFM95_RESET_On;
    TRX.RESET_Off = RFM95_RESET_Off;

    TRX.RESET_On();
    // wait(10)
    TRX.RESET_Off();
    // wait(10)
    TRX.BaseFrequency = BaseFreq;
    TRX.ChannelSpacing = ChanSpace;
    TRX.FrequencyCorrection = 0;
    TRX.Configure(0, OGN_SYNC);
    TRX.WriteMode(RH_RF95_MODE_STDBY);
}

void TX_OGN_packet_test(uint8_t *packet) {
    TRX.WriteMode(RH_RF95_MODE_STDBY);
    TRX.WriteTxPower(14);
    TRX.WriteSYNC(8, 7, OGN_SYNC);
    TRX.ClearIrqFlags();
    TRX.WritePacket(packet);
    //    TRX.WritePacket(
    //        (const
    //        uint8_t*)"qwertyuiyuiopqwertyuio"
    //        "p",
    //        15 * 10);
    TRX.WriteMode(RH_RF95_MODE_TX);
    TRX.WriteMode(RH_RF95_MODE_STDBY);
    TRX.WriteTxPowerMin();
    TRX.WriteSYNC(7, 7, OGN_SYNC);
}

int RX_OGN_packet_test(uint8_t *pBuffer) {
    // uint16_t
    uint8_t errBuffer[26];

    TRX.WriteMode(RH_RF95_MODE_STDBY);
    // TODO config before switch RX mode?
    TRX.WriteMode(RH_RF95_MODE_RX);
    // Wait for RxReady
    while (TRX.ReadIrqFlags() & RH_RF95_IRQ_RxReady)
        ;
    if (TRX.ReadIrqFlags() & RH_RF95_IRQ_PayloadReady) {  // on payload ready flag
        TRX.ReadPacket(pBuffer, (uint8_t *)errBuffer);

        // return (TRX.ReadIrqFlags())
    }
    return 0;
    // wait until receive
    // PayloadReady flag
}
