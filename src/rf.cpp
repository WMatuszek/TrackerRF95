
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

#include "fifo.h"
#include "format.h"
#include "rf.h"
#include "rfm95.h"
#include "spi.h"
#include "tim.h"
#include "uart2.h"

// RFM device DIO pin map to GPIO B
#define RFM95_DIO0 GPIO_Pin_3
#define RFM95_DIO4 GPIO_Pin_4

inline void RFM95_RESET_On(void) {
    GPIO_SetBits(GPIOB, GPIO_Pin_5);
}
inline void RFM95_RESET_Off(void) {
    GPIO_ResetBits(GPIOB, GPIO_Pin_5);
}

inline bool RFM95_DIO0_isOn(void) {
    return (GPIO_ReadInputDataBit(GPIOB, RFM95_DIO0) == Bit_SET);
}

inline bool RFM95_DIO4_isOn(void) {
    return (GPIO_ReadInputDataBit(GPIOB, RFM95_DIO4) == Bit_SET);
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

    // #TODO init DIO0-5 pins
    GPIO_InitStructure.GPIO_Pin = (RFM95_DIO0 | RFM95_DIO4);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  // Mode - input, pulled low
    GPIO_Init(GPIOB, &GPIO_InitStructure);

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
    TRX.Select = SPI1_Select;
    TRX.Deselect = SPI1_Deselect;
    TRX.TransferByte = SPI1_TransferByte;
    TRX.DIO0_isOn = RFM95_DIO0_isOn;
    TRX.DIO4_isOn = RFM95_DIO4_isOn;
    TRX.RESET_On = RFM95_RESET_On;
    TRX.RESET_Off = RFM95_RESET_Off;

    TRX.RESET_On();
    TIM3_DelayMS(10);  // #TODO how long the delay was supposed to be?
    TRX.RESET_Off();
    TIM3_DelayMS(10);
    TRX.BaseFrequency = BaseFreq;
    TRX.ChannelSpacing = ChanSpace;
    TRX.FrequencyCorrection = 0;
    TRX.Configure(0, OGN_SYNC);
    TRX.WriteMode(RH_RF95_MODE_STDBY);
}

/*
 * Transmits packet of default lenght of 26
 * Return 1 on success - PacketSent IRQ active
 */
uint8_t RFM95_TransmitPacket(uint8_t *txPakData) {
    uint8_t retCode = 0;
    TRX.WriteMode(RH_RF95_MODE_STDBY);  // go standby mode for TX setup
    TRX.WriteTxPower(14);               // write TX power
    TRX.WriteSYNC(8, 7, OGN_SYNC);      // write complete SYNC seq
    TRX.ClearIrqFlags();
    TRX.WritePacket(txPakData);

    TRX.WriteMode(RH_RF95_MODE_TX);  // mode to TX

    // Wait for transmission to end
    for (uint8_t waitCnt = 10; waitCnt; --waitCnt) {
        TIM3_DelayUS(10);
        uint8_t mode = TRX.ReadMode();
        uint16_t flags = TRX.ReadIrqFlags();
        if (mode != RH_RF95_MODE_TX) break;  // mode no longer TX
        if (RFM95_DIO0_isOn()) {             // PacketSent IRQ active
            retCode = 1;
            break;
        }
    }

    TRX.WriteMode(RH_RF95_MODE_STDBY);  // mode back to STDBY
    TRX.WriteTxPowerMin();              // write min power (?)
    TRX.WriteSYNC(7, 7, OGN_SYNC);      // write incomplete SYNC seq for RX mode (#TODO why?)
    return retCode;
}

/*
 * Receives packet of default lenght of 26
 * Returns if PayloadReady IRQ not active
 */
uint8_t RFM95_ReceivePacket(uint8_t *pakData, uint8_t *pakErr) {
    if (RFM95_DIO0_isOn() == false) return 0;  // PayloadReady IRQ not active

    TRX.ReadPacket(pakData, pakErr);  // read packet

    // #TODO handle pakError check
    // LDPC_Check(pakData);

    return 1;
}

RFM95_RX_Stats RFM95_Receive_TEST(uint8_t *pakData, uint8_t *expectedData, uint16_t rcvSlot_ms) {
    uint8_t tmp;
    uint8_t errBuffer[26];
    struct RFM95_RX_Stats stats;

    TRX.WriteMode(RH_RF95_MODE_STDBY);  // mode to STDBY
    // TODO config before switch RX mode?
    TRX.WriteMode(RH_RF95_MODE_RX);  // mode to RX
    TIM3_DelayMS(1);                 // Wait for RxReady (default BW -> 600us)

    // Wait for rcvSlot ms while receiving packets
    // #TODO except not at all tht many ms because UART writes
    for (rcvSlot_ms; rcvSlot_ms; --rcvSlot_ms) {
        tmp = RFM95_ReceivePacket(pakData, errBuffer);

        // Check for byte errors
        if (tmp) {
            Format_String(UART2_Write, "RX ");  // Received packet to UART2
            Format_Bytes(UART2_Write, pakData, 26);

            uint8_t errThisPacket = 0;
            for (int i = 0; i < 26; ++i) {
                if (pakData[i] != expectedData[i]) errThisPacket++;
            }
            if (errThisPacket) {
                Format_String(UART2_Write, "ERR ");  // Byte error cnt to UART2
                Format_Hex(UART2_Write, errThisPacket);
            }

            Format_String(UART2_Write, "\n");  // new line to UART2
            stats.byteErrorCnt += errThisPacket;
        }
        stats.rxPacketCnt += tmp;
        TIM3_DelayMS(1);
    }

    TRX.WriteMode(RH_RF95_MODE_STDBY);

    return stats;
}
