#include "interrupt.h"

uint8_t last_state, FLAG_DELAY, FLAG_BUTTON;
uint32_t delay_count;

void EXTI15_10_IRQHandler(void)
{
    SET_BIT(EXTI->PR, EXTI_PR_PR11);
    SET_BIT(EXTI->PR, EXTI_PR_PR13);
    if (READ_BIT(GPIOC->IDR, GPIO_IDR_ID11))
    {
        FLAG_BUTTON = 1;
        if (button_delay_tim_count >= 20)
        {
            btnCount1++;
            button_delay_tim_count = 0;
        }
        FLAG_BUTTON = 0;
    }
    if (READ_BIT(GPIOC->IDR, GPIO_IDR_ID12))
    {
        btnCount2++;
        flag = !flag;
    }
}

void EXTI4_IRQHandler(void)
{
    SET_BIT(EXTI->PR, EXTI_PR_PR4);
    btnCount1++;
    flag = !flag;
}

void SysTick_Handler(void)
{
    if (FLAG_BUTTON)
    {
        button_delay_tim_count++;
    }
    global_systick_tim_count++;
    second_tim_count++;
    if (FLAG_DELAY)
    {
        delay_count++;
    }
}

void delay(uint32_t del)
{
    FLAG_DELAY = 1;
    while (del >= delay_count)
    {
    }
    delay_count = 0;
    FLAG_DELAY = 0;
}

// void delay(uint32_t delay)
// {
//     FLAG_DELAY = 1;
//     while (delay >= delay_tim_count)
//     {
//     }
//     delay_tim_count = 0;
//     FLAG_DELAY = 0;
// }