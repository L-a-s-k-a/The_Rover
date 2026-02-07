#pragma once
#ifndef INIT_H
#define INIT_H

#include "../../CMSIS/Devices/STM32F4xx/Inc/stm32f4xx.h"
#include "../../CMSIS/Devices/STM32F4xx/Inc/STM32F407xx/stm32f407xx.h"

void RCC_Init(void);
void ITR_Init(void);
void GPIO_Init(void);
void SysTick_Init(void);
void TIM_PWM_Init(void);
void ADC_Init_Polling(void);
// void TIM_ENCODER_Init(void);
// void Enc_Trig_Init(void);

#endif
