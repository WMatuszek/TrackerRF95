#include "tim.h"

#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

void TIM2_setup() {
    NVIC_InitTypeDef nvic;
    TIM_TimeBaseInitTypeDef tim;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Prescaler = 64000 - 1;
    tim.TIM_Period = 1000 - 1;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void TIM3_setup() {
    TIM_TimeBaseInitTypeDef tim;

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_CounterMode = TIM_CounterMode_Down;
    // tim.TIM_ClockDivision = TIM_CKD_DIV2;
    tim.TIM_Prescaler = (SystemCoreClock) / 1000000;
    tim.TIM_Period = 1;
    TIM_TimeBaseInit(TIM3, &tim);
}

void TIM3_DelayUS(uint16_t us) {
    TIM_SetCounter(TIM3, us);
    TIM_Cmd(TIM3, ENABLE);
    while (TIM_GetCounter(TIM3) != 0)
        ;
    TIM_Cmd(TIM3, DISABLE);
}

void TIM3_DelayMS(uint16_t ms) {
    while (ms--) {
        TIM3_DelayUS(1000);
    }
}
