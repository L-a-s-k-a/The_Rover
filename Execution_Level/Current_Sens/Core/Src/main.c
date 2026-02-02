// #include "../Inc/init.h"

#include "interrupt.h"
#include "../../CMSIS/Devices/STM32F4xx/Inc/stm32f4xx.h"
#include "../../CMSIS/Devices/STM32F4xx/Inc/STM32F429ZI/stm32f429xx.h"

volatile uint8_t btnCount1 = 0, btnCount2 = 0, flag = 0;
volatile uint32_t global_systick_tim_count = 0;
volatile uint16_t button_delay_tim_count, second_tim_count;
volatile uint32_t second = 0;

uint16_t result, chislo_a, chislo_b;
int32_t variable, motor_speed, something_else;

int main(void)
{
    RCC_Init();
    ITR_Init();
    GPIO_Init();
    SysTick_Init();

    while (1)
    {
        // SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS7);
        // second++;
        // chislo_a++;
        // delay(1000);
        // SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR7);
        // second++;
        // chislo_b++;
        // delay(1000);
        if(testik){
            chislo_a = 0;
        }
        if (second_tim_count >= 1000)
        {
            second++;
            second_tim_count = 0;
        }
        if (global_systick_tim_count >= 500)
        {
            chislo_b++;
            global_systick_tim_count = 0;
        }
        if (second % 2)
        {
            SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS7);
        }
        else
        {
            SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR7);
        }

        if (READ_BIT(GPIOC->IDR, GPIO_IDR_ID12))
        {
            chislo_a++;
            SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS14);
        }
        else
            SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR14);

        result = chislo_a + chislo_b;
    }
}

// int main(void)
// {
//     GPIO_Init_With_Myself_Macros();
//     GPIO_Init_CMSIS();

//     while (1)
//     {
//         /*---------------Чтение кнопки с помощью CMSIS------------*/
//         if(READ_BIT(GPIOG->IDR, GPIO_IDR_ID1)){
//             SET_BIT(GPIOE->BSRR, GPIO_BSRR_BS0); // Включение светодидоа PE0
//         }
//         else{
//             SET_BIT(GPIOE->BSRR, GPIO_BSRR_BR0); // Отключение светодидоа РЕ0
//         }
//         /*------Чтение кнопки с помощью собственных макросов------*/
//         if(BIT_READ(GPIOC_IDR, GPIO_PIN_13)){
//             BIT_SET(GPIOB_BSRR, GPIO_PIN_SET_7); // Включение светодидоа РВ7, регистр BSRR
//         }
//         else {
//             BIT_SET(GPIOB_BSRR, GPIO_PIN_RESET_7); // Отключение светодидоа РВ7, регистр BSRR
//         }

//     }
// }

/*8EM52 китайская*/
// int main(void)
// {
//     GPIO_Init_Memory();
//     GPIO_Init_CMSIS();

//     while (1)
//     {
//         if(READ_BIT(GPIOC->IDR, GPIO_IDR_ID13)){
//             *(uint32_t *)(0x40020400UL + 0x18UL) |= 0x80UL;     // Включение синего светодидоа, регистр BSRR
//             SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR14);               // Отключение красного светодиода
//         }
//         else {
//             *(uint32_t *)(0x40020400UL + 0x18UL) |= 0x800000UL; // Отключение светодидоа, регистр BSRR
//             SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS14);               // Включение красного светодиода
//         }

//         if(READ_BIT(GPIOG->IDR, GPIO_IDR_ID3)){
//             SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS0);                //Включение зеленого светодиода
//             SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR7);                //Отключение отдельного светодиода
//         }
//         else {
//             SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR0);                //Отключение зеленого светодиода
//             SET_BIT(GPIOD->BSRR, GPIO_BSRR_BS7);                //Включение отдельного светодиода
//         }

//     }
// }

/*8Е31*/
// int main(void)
// {
//     RCC_Init();

//     SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN);

//     SET_BIT(GPIOB->MODER, GPIO_MODER_MODER7_0);
//     SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR7);

//     /*--------Настройка работы порта PC9 в качестве MCO2--------*/
//     SET_BIT(GPIOC->MODER, GPIO_MODER_MODER9_1);              //Настраиваем пин на альтернативный режим
//     SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED9);           //Настраиваем пин на максимальную скорость работы
//     MODIFY_REG(GPIOC->AFR[1], GPIO_AFRH_AFSEL9_Msk, 0x00UL); //Выбираем тип альтернативной функции – Выход MCO2

//     /*--------Настройка работы порта PA8 в качестве MCO1--------*/
//     SET_BIT(GPIOA->MODER, GPIO_MODER_MODE8_1);
//     SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED8);
//     CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL8);

//     while (1)
//     {
//         SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS7);
//     }
// }

/* 8TM51 подгруппа со старостой*/
// #include "../Inc/init.h"

// int main(void)
// {
//     GPIO_Init_With_Miself_Macros();

//     while (1)
//     {
//         if (BIT_READ(GPIOC_IDR, GPIO_PIN_13))
//         {
//             BIT_SET(GPIOB_BSRR, GPIO_PIN_SET_7); // Включение светодиода PB7, регистр BSRR
//         }
//         else
//         {
//             BIT_SET(GPIOB_BSRR, GPIO_PIN_RESET_7);
//         }
//     }
// }