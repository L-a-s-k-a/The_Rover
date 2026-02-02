#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "init.h"

extern volatile uint8_t btnCount1, btnCount2, flag;
extern volatile uint32_t global_systick_tim_count;
extern volatile uint16_t button_delay_tim_count, second_tim_count;

void delay(uint32_t del);

#endif