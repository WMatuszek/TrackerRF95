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

uint8_t testPacket[26] = {0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x48, 0x65, 0x6C,
                          0x6C, 0x6F, 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x00};

int main(int argc, char* argv[]) {
    uint8_t rxBuffer[26];
    struct RFM95_RX_Stats rxStats;

    gpio_setup();
    // TIM2_setup();
    TIM3_setup();

    // SPI1_Configuration();
    // StartRFchip();
    // UART1_Configuration(9600);

    UART2_Configuration(9600);
    UART2_WriteStr((char*)"Hello\n");

    GPIO_SetBits(GPIOA, GPIO_Pin_5);

    while (1) {
        TIM3_DelayMS(2000);
        GPIO_SetBits(GPIOA, GPIO_Pin_5);
        TIM3_DelayMS(2000);
        GPIO_ResetBits(GPIOA, GPIO_Pin_5);
        // rxStats = RFM95_Receive_TEST(rxBuffer, testPacket, 100);
        // #TODO use the stats
    }
}

extern "C" void TIM2_IRQHandler() {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        if (TEST_MODE == TEST_TX) {
            // RFM95_TransmitPacket(testPacket);
        }
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        GPIOA->ODR = (GPIOA->ODR) ^ (1 << 5);
        // GPIOC->ODR = (GPIOC->ODR) ^ (1 << 13);
    }
}

#pragma GCC diagnostic pop
