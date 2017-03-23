#ifndef spi_h
#define spi_h

#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

void SPI1_Configuration(void);
uint8_t SPI1_TransferByte(uint8_t Byte);

inline void SPI1_Select(void) {
    GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_RESET);
}
inline void SPI1_Deselect(void) {
    GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);
}

inline void SPI_Select(uint8_t Select) {
    if (Select)
        SPI1_Select();
    else
        SPI1_Deselect();
}

#ifdef __cplusplus
}
#endif

#endif
