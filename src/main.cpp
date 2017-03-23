#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "gpio.h"
#include "rf.h"
#include "rfm95.h"
#include "spi.h"
#include "tim.h"
#include "uart1.h"
#include "uart2.h"

// # TODO move somewhere
class OGN_Packet  // Packet structure for the OGN tracker
{
 public:
    union {
        uint32_t Word[7];  // OGN packet as 32-bit words
        uint8_t Byte[26];  // OGN packet as  8-bit bytes

        struct  // OGN packet as Header+Position+FEC
        {
            uint32_t Header;  //    ECRR PMTT AAAA AAAA AAAA AAAA AAAA AAAA
                              // E=Emergency, C=enCrypt/Custom, RR=Relay count, P=Parity, M=isMeteo/Telemetry, TT=address Type, AA..=Address:24-bit
                              // When enCrypt/Custom is set the data (position or whatever) can only be decoded by the owner
                              // This option is indented to pass any type of custom data not foreseen otheriwse

            uint32_t Position[4];  // 0: QQTT TTTT LLLL LLLL LLLL LLLL LLLL LLLL  QQ=fix Quality:2, TTTTTT=time:6, LL..=Latitude:20
                                   // 1: MBDD DDDD LLLL LLLL LLLL LLLL LLLL LLLL  F=fixMode:1 B=isBaro:1, DDDDDD=DOP:6, LL..=Longitude:20
                                   // 2: RRRR RRRR SSSS SSSS SSAA AAAA AAAA AAAA  RR..=turn Rate:8, SS..=Speed:10, AA..=Alt:14
            // 3: BBBB BBBB YYYY PCCC CCCC CCDD DDDD DDDD  BB..=Baro altitude:8, YYYY=AcftType:4, P=Stealth:1, CC..=Climb:9, DD..=Heading:10

            // meteo/telemetry types: Meteo conditions, Thermal wind/climb, Device telemetry, Precise time,

            // meteo report would transmit: Humidity, Barometric pressure, Temperature, wind Speed/Direction
            // 2: HHHH HHHH SSSS SSSS SSAA AAAA AAAA AAAA
            // 3: TTTT TTTT YYYY BBBB BBBB BBDD DDDD DDDD  YYYY = report tYpe (meteo, thermal, water level, other telemetry)

            uint32_t FEC[2];  // Gallager code: 48 check bits for 160 user bits
        };
    };
};

/*
 * ARDUINO	PIN		FUNCTION
 * D13		PA5		SPI1_CLK -> RFM95_CLK
 * D12		PA6		SPI1_MOSI -> RFM95_MOSI
 * D11		PA7		SPI1_MISO <- RFM95_MISO
 * D10		PB6		-> RFM95_CS
 * D9		PC7
 * D8		PA9		UART1_TX -> GPS_RX
 * D7		PA8
 * D6		PB10
 * D5		PB4		-> GPS_ENABLE
 * D4		PB5		-> RFM95_RESET
 * D3		PB3		<- GPS_PPS
 * D2		PA10	UART1_RX <- GPS_TX
 * D1		PA2		UART2_TX -> ST_LINK
 * D0		PA3		UART2_RX <- ST_LINK
 */

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#define TEST_RX 1
#define TEST_TX 2

const int TEST_MODE = TEST_TX;

uint8_t packetPlaceholder[26];  // #TODO init with proper payload

int main(int argc, char* argv[]) {
    uint8_t packetBuffer[26];

    gpio_setup();
    TIM2_setup();
    // SPI1_Configuration();
    // StartRFchip();
    // UART1_Configuration(9600);
    UART2_Configuration(9600);
    UART2_WriteStr("Hello");
    GPIO_SetBits(GPIOA, GPIO_Pin_5);

    while (1) {
        // RX_OGN_packet_test((uint8_t*)packetBuffer);
    }
}

extern "C" void TIM2_IRQHandler() {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        if (TEST_MODE == TEST_TX) {
            // TX_OGN_packet_test((uint8_t*)packetPlaceholder);
        }
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        GPIOA->ODR = (GPIOA->ODR) ^ (1 << 5);
        // GPIOC->ODR = (GPIOC->ODR) ^ (1 << 13);
    }
}

#pragma GCC diagnostic pop
