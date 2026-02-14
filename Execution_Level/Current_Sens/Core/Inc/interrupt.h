#pragma once
#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "init.h"

extern volatile uint16_t adcInter;
extern uint16_t GlobalTickCount;

void EXTI15_10_IRQHandler(void);
void SysTick_Handler(void);
void TIM3_IRQHandler(void);
void TIM2_IRQHandler(void);
void delay(int del);

#endif