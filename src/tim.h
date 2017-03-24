#ifndef tim_h
#define tim_h

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

void TIM2_setup(void);
void TIM3_setup(void);

void TIM3_DelayUS(uint16_t us);
void TIM3_DelayMS(uint16_t ms);

#ifdef __cplusplus
}
#endif

#endif
