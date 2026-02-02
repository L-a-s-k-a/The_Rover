#include "../Inc/init.h"

static uint8_t testik;

/* группа 8Е32, все*/
void RCC_Init(void)
{
    MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U);
    CLEAR_REG(RCC->CFGR);
    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET)
        ;
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET)
        ;
    CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON);
    while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET)
        ;
    CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);

    SET_BIT(RCC->CR, RCC_CR_HSEON); // Включение внешнего источника тактирования высокой частоты
    while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET)
        ;
    SET_BIT(RCC->CR, RCC_CR_CSSON); // Включение Clock Security

    CLEAR_REG(RCC->PLLCFGR);
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_HSE); // Источник тактирвоания HSE
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_2);     // Деление источника тактирования на 4 (PLLM)
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_5 |
                              RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_8); // Настройка умножения на 360 (PLLN)
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_0);     // Деление частоты после умножения на 4 (PLLP)
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1 |
                              RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLQ_3); // Деление частоты после умножения на 15 (PLLQ)

    SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);                         // В качестве системного тактирования выбран PLL
    SET_BIT(RCC->CFGR, RCC_CFGR_HPRE_DIV1);                      // Предделитель шины AHB1 настроен на 1 (без деления)
    SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV4);                     // Предделитель шины APB1 настроен на 4 (45МГц)
    SET_BIT(RCC->CFGR, RCC_CFGR_PPRE2_DIV2);                     // Предделитель шины APB2 настроен на 2 (90МГц)
    SET_BIT(RCC->CFGR, RCC_CFGR_MCO1);                           // Нстройка вывода частоты PLL на MCO1
    SET_BIT(RCC->CFGR, RCC_CFGR_MCO1PRE_2 | RCC_CFGR_MCO1PRE_1); // Предделитель 4 для вывода на MCO1
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_MCO2);                         // Настройка вывода частоты SYSCLK на MCO2
    SET_BIT(RCC->CFGR, RCC_CFGR_MCO2PRE_2 | RCC_CFGR_MCO2PRE_1); // Предделитель 4 для вывода на MCO2

    SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY_5WS); // Установка 5 циклов ожидания для FLASH памяти

    SET_BIT(RCC->CR, RCC_CR_PLLON); // Включение блока PLL
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET);
}

void ITR_Init(void)
{
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);

    SET_BIT(SYSCFG->EXTICR[2], SYSCFG_EXTICR3_EXTI11_PC);
    SET_BIT(EXTI->IMR, EXTI_IMR_IM11);
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR11);
    CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_TR11);

    SET_BIT(SYSCFG->EXTICR[3], SYSCFG_EXTICR4_EXTI12_PC);
    SET_BIT(EXTI->IMR, EXTI_IMR_IM12);
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR12);
    CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_TR12);

    NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    SET_BIT(SYSCFG->EXTICR[1], SYSCFG_EXTICR2_EXTI4_PA);
    SET_BIT(EXTI->IMR, EXTI_IMR_IM4);
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR4);
    CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_TR4);

    NVIC_SetPriority(EXTI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(EXTI4_IRQn);
}

void SysTick_Init(void){
    CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
    SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
    SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk);
    MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, (180000 - 1) << SysTick_LOAD_RELOAD_Pos);
    MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, (180000 - 1) << SysTick_VAL_CURRENT_Pos);
    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
}

void GPIO_Init(void)
{
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOEEN);

    SET_BIT(GPIOB->MODER, GPIO_MODER_MODER7_0);
    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR7);
    SET_BIT(GPIOB->MODER, GPIO_MODER_MODER14_0);
    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR14);
    /*--------Настройка работы порта PC9 в качестве MCO2--------*/
    SET_BIT(GPIOC->MODER, GPIO_MODER_MODER9_1);
    SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED9);
    CLEAR_BIT(GPIOC->AFR[1], GPIO_AFRH_AFSEL9);

    /*--------Настройка работы порта PA8 в качестве MCO1--------*/
    SET_BIT(GPIOA->MODER, GPIO_MODER_MODER8_1);
    SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED8);
    CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL8);
}


/*Подгруппа без стаорсты 8ТМ51*/
/*
void GPIO_Init_With_Myself_Macros(void){
    BIT_SET(RCC_AHB1ENR, RCC_GPIOB_EN | RCC_GPIOC_EN);  // Включение тактирования для периферии GPIOB и GPIOC, регистр AHB1ENR

    BIT_SET(GPIOB_MODER, GPIO_OUTPUT_MODE_PIN_7);       // Настройка режима работы порта на выход, регистр MODER
    BIT_CLEAR(GPIOB_OTYPER, GPIO_PP_PIN_7);             // Настройка режима выхода пина PB7 на push-pull, регистр OTYPER
    BIT_SET(GPIOB_OSPEEDR, GPIO_SPEED_MEDIUM_PIN_7);    // Настройка скорости работы порта на среднюю, регистр OSPEEDR

    BIT_SET(GPIOB_BSRR, GPIO_PIN_RESET_7);              // Предварительное отключение светодидоа, регистр BSRR
}

void GPIO_Init_CMSIS(void){
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOGEN);

    SET_BIT(GPIOE->MODER, GPIO_MODER_MODER0_0);         //Настройка режима работы пина РE0 на выход
    SET_BIT(GPIOE->OSPEEDR, GPIO_OSPEEDR_OSPEED0_0);    //Настройка скорости работы пина PE0 на среднюю скорость
    SET_BIT(GPIOE->BSRR, GPIO_BSRR_BR0);                //Выключение светодиода на пине РЕ0
}*/

/*8e32 староста*/
// void RCC_Init(void)
// {
//     MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U);
//     CLEAR_REG(RCC->CFGR);
//     while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET)
//         ;
//     CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
//     while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET)
//         ;
//     CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON);
//     while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET)
//         ;
//     CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);

//     SET_BIT(RCC->CR, RCC_CR_HSEON);
//     while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET)
//         ;
//     SET_BIT(RCC->CR, RCC_CR_CSSON);

//     CLEAR_REG(RCC->PLLCFGR);
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_HSE);
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_2);
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_3 |
//                               RCC_PLLCFGR_PLLN_5 |
//                               RCC_PLLCFGR_PLLN_6 |
//                               RCC_PLLCFGR_PLLN_8);
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_0);
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_0 |
//                               RCC_PLLCFGR_PLLQ_1 |
//                               RCC_PLLCFGR_PLLQ_2 |
//                               RCC_PLLCFGR_PLLQ_3);

//     SET_BIT(RCC->CFGR, RCC_CFGR_SW_1);
//     SET_BIT(RCC->CFGR, RCC_CFGR_HPRE_DIV2);
//     SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV4);
//     SET_BIT(RCC->CFGR, RCC_CFGR_PPRE2_DIV2);
//     SET_BIT(RCC->CFGR, RCC_CFGR_MCO1);
//     SET_BIT(RCC->CFGR, RCC_CFGR_MCO1PRE_2 | RCC_CFGR_MCO1PRE_1);
//     CLEAR_BIT(RCC->CFGR, RCC_CFGR_MCO2);
//     SET_BIT(RCC->CFGR, RCC_CFGR_MCO2PRE_2 | RCC_CFGR_MCO2PRE_1 | RCC_CFGR_MCO2PRE_0);

//     SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY_5WS);

//     SET_BIT(RCC->CR, RCC_CR_PLLON);
//     while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET)
//         ;
//     CLEAR_BIT(RCC->CR, RCC_CR_HSION);
// }

// void IRQ_Inint(void)
// {
//     SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);

//     SET_BIT(SYSCFG->EXTICR[3], SYSCFG_EXTICR4_EXTI12_PC);
//     SET_BIT(EXTI->IMR, EXTI_IMR_IM12);
//     SET_BIT(EXTI->RTSR, EXTI_RTSR_TR12);
//     CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_TR12);

//     NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
//     NVIC_EnableIRQ(EXTI15_10_IRQn);
// }

/*8EM52 китайская*/
// void GPIO_Init_Memory(void)
// {
//     *(uint32_t *)(0x40023800UL + 0x30UL) |= 0x02UL + 0x04UL; // Включение тактирвоания для периферии GPIOB, регистр AHB1ENR

//     *(uint32_t *)(0x40020400UL + 0x00UL) |= 0x4000UL; // Настройка режима работы порта на выход, регистр MODER
//     *(uint32_t *)(0x40020400UL + 0x04UL) &= ~0x80UL;  // Настройка режима выхода пина PB7 на push-pull, регистр OTYPER
//     *(uint32_t *)(0x40020400UL + 0x08UL) |= 0x4000UL; // Настройка скорости работы порта на среднюю, регистр OSPEEDR

//     *(uint32_t *)(0x40020400UL + 0x18UL) |= 0x800000UL; // Предварительное отключение светодидоа, регистр BSRR
// }
/*8EM52 китайская*/
// void GPIO_Init_CMSIS(void)
// {
//     SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN |
//                           RCC_AHB1ENR_GPIOCEN |
//                           RCC_AHB1ENR_GPIOGEN |
//                           RCC_AHB1ENR_GPIODEN);   /*Включение тактирования на порт GPIOB
//                                                    *на порт GPIOC, на порт GPIOG, на порт GPIOD
//                                                    */

//     SET_BIT(GPIOB->MODER, GPIO_MODER_MODER14_0);                        //Настройка режима работы пина PB14 на выход
//     SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR14);                               //Установка 0 (Нуля) на выходе PB14

//     SET_BIT(GPIOB->MODER, GPIO_MODER_MODER0_0);                         //Настройка режима работы пина PB0 на выход
//     SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR0);                                //Установка 0 (Нуля) на выходе PB0

//     SET_BIT(GPIOD->MODER, GPIO_MODER_MODER7_0);                         //Настройка режима работы пина PD7 на выход
//     SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR7);                                //Установка 0 (Нуля) на выходе PD7
// }

/*ГРуппа 8Е31*/
// void RCC_Init(void)
// {
//     MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U);
//     CLEAR_REG(RCC->CFGR);
//     while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET);
//     CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
//     while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET);
//     CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON);
//     while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET);
//     CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);

//     SET_BIT(RCC->CR, RCC_CR_HSEON);                                 //Включение внешнего источника тактирования высокой частоты
//     while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET);
//     SET_BIT(RCC->CR, RCC_CR_CSSON);                                 //Включение Clock Security

//     CLEAR_REG(RCC->PLLCFGR);
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_HSE);                  //Источник тактирвоания HSE
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_2);                      //Деление источника тактирования на 4 (PLLM)
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_5 |
//                           RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_8); //Настройка умножения на 360 (PLLN)
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_1 | RCC_PLLCFGR_PLLP_0); //Деление частоты после умножения на 4 (PLLP)
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1 |
//                           RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLQ_3); //Деление частоты после умножения на 15 (PLLQ)

//     SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);                            //В качестве системного тактирования выбран PLL
//     SET_BIT(RCC->CFGR, RCC_CFGR_HPRE_DIV1);                         //Предделитель шины AHB1 настроен на 1 (без деления)
//     SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV4);                        //Предделитель шины APB1 настроен на 4 (45МГц)
//     SET_BIT(RCC->CFGR, RCC_CFGR_PPRE2_DIV2);                        //Предделитель шины APB2 настроен на 2 (90МГц)
//     SET_BIT(RCC->CFGR, RCC_CFGR_MCO1);                              //Нстройка вывода частоты PLL на MCO1
//     SET_BIT(RCC->CFGR, RCC_CFGR_MCO1PRE_2 | RCC_CFGR_MCO1PRE_1);    //Предделитель 4 для вывода на MCO1
//     CLEAR_BIT(RCC->CFGR, RCC_CFGR_MCO2);                            //Настройка вывода частоты SYSCLK на MCO2
//     SET_BIT(RCC->CFGR, RCC_CFGR_MCO2PRE_2 | RCC_CFGR_MCO2PRE_1 | RCC_CFGR_MCO2PRE_0);    //Предделитель 4 для вывода на MCO2

//     SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY_5WS);                     //Установка 5 циклов ожидания для FLASH памяти

//     SET_BIT(RCC->CR, RCC_CR_PLLON);                                 //Включение блока PLL
//     while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET);
// }

// void RCC_Init(void)
// {
//     MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U);
//     CLEAR_REG(RCC->CFGR);
//     while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET);
//     CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
//     while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET);
//     CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON);
//     while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET);
//     CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);

//     SET_BIT(RCC->CR, RCC_CR_HSEON);
//     while(READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET);
//     SET_BIT(RCC->CR, RCC_CR_CSSON);
//     SET_BIT(RCC->CR, RCC_CR_HSEBYP);
//     CLEAR_BIT(RCC->CR, RCC_CR_HSION);

//     CLEAR_REG(RCC->PLLCFGR);
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_HSE);                   //Источник тактирвоания HSE
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_2);                       //Деление источника тактирования на 4
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_5 |
//                           RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_8);  //Настройка умножения на 360
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_0);                       //Делитель для выхода PLL(PLLCLK) на 4
//     SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1 |
//                           RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLQ_3);  //Делитель для USB на 1111

//     SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);                             //Источник системного тактирования -> выход PLL
//     CLEAR_BIT(RCC->CFGR, RCC_CFGR_HPRE_DIV1);                        //Делитель AHB1 -> 0
//     SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV4);                         //Делитель AРB1 -> 4 (45 МГц)
//     SET_BIT(RCC->CFGR, RCC_CFGR_PPRE2_DIV2);                         //Делитель AРB2 -> 2 (90 МГц)
//     CLEAR_BIT(RCC->CFGR, RCC_CFGR_MCO2);                             //Выход МСО2 -> Sysclk
//     SET_BIT(RCC->CFGR, RCC_CFGR_MCO2PRE_0 | RCC_CFGR_MCO2PRE_1 | RCC_CFGR_MCO2PRE_2);     //Делитель для МСО2 -> 4

//     SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY_5WS);

//     SET_BIT(RCC->CR, RCC_CR_PLLON);
//     while(READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET);
// }



// void GPIO_Init_Memory(void)
// {
//     *(uint32_t *)(0x40023800UL + 0x30UL) |= 0x06UL;

//     *(uint32_t *)(0x40020400UL + 0x00UL) |= 0x4000UL; // Настройка пина PB7 на вывод, регистр MODER
//     *(uint32_t *)(0x40020400UL + 0x08UL) |= 0x4000UL; // Натсройка скорости пина PB7 на среднюю, регистр OSPEEDR
//     *(uint32_t *)(0x40020400UL + 0x0CUL) |= 0x00UL;   // Настройка подтягивающих/стягивающих резисторов, регистр PUPDR

//     *(uint32_t *)(0x40020400UL + 0x18UL) |= 0x800000UL; // Выключение светодиода PB7, регистр BSRR
// }

// void GPIO_Init_With_Miself_Macros(void)
// {
//     BIT_SET(RCC_AHB1ENR, RCC_GPIOB_EN | RCC_GPIOC_EN);

//     BIT_SET(GPIOB_MODER, GPIO_OUTPUT_MODE_PIN_7);
//     BIT_SET(GPIOB_OSPEEDR, GPIO_SPEED_MED_PIN_7);
//     BIT_SET(GPIOB_PUPDR, GPIO_OFF);

//     BIT_SET(GPIOB_BSRR, GPIO_PIN_RESET_7);
// }

// void GPIO_Init_Memory(void){
//     *(uint32_t *)(0x40023800UL + 0x30UL) |= 0x06UL;     //Включение тактирования на периферии GPIOB и GPIOC, регистр AHB1ENR

//     *(uint32_t *)(0x40020400UL + 0x00UL) |= 0x4000UL;   //Настройка пина PB7 на вывод, регистр MODER
//     *(uint32_t *)(0x40020400UL + 0x04UL) |= 0x00UL;     //Настройка режима работы выхода на push-pull, регистр OTYPER
//     *(uint32_t *)(0x40020400UL + 0x08UL) |= 0x4000UL;   //Настройка скорости работы вывода PB7, регистр OSPEEDR
//     *(uint32_t *)(0x40020400UL + 0x18UL) |= 0x80000UL;  //Предварительное выключение светодиода, регистр BSRR, бит BR7
// }

// void GPIO_Init_With_Myself_Macros(void){
//     RCC_AHB1ENR |= RCC_GPIOB_EN | RCC_GPIOC_EN; //Включение тактирования на периферии GPIOB и GPIOC, регистр AHB1ENR

//     BIT_SET(GPIOB_MODER,   GPIO_PIN_OUT_7);     //Настройка пина PB7 на вывод, регистр MODER
//     BIT_SET(GPIOB_OTYPER,  GPIO_OFF);           //Настройка режима работы выхода на push-pull, регистр OTYPER
//     BIT_SET(GPIOB_OSPEEDR, GPIO_PIN_Med_7);     //Настройка скорости работы вывода PB7, регистр OSPEEDR
//     BIT_SET(GPIOB_BSRR,    GPIO_PIN_RESET_7);   //Предварительное выключение светодиода, регистр BSRR, бит BR7
// }

// void GPIO_Init_CMSIS(void){
//     SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);

//     SET_BIT(GPIOB->MODER, GPIO_MODER_MODER14_0);  //Настройка пина PB14 на вывод, регистр MODER
//     CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT14); //Настройка режима работы выхода на push-pull, регистр OTYPER
//     SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR14_0); //Настройка скорости работы вывода PB14, регистр OSPEEDR
//     SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR14); //Предварительное выключение светодиода, регистр BSRR, бит BR14
// }

// void GPIO_Init_CMSIS(void){
//     SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);

//     SET_BIT(GPIOB->MODER, GPIO_MODER_MODER14_0);
//     SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED14_0);
//     SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR14);
// }

// void GPIO_Init_Memory(void){
//     *(uint32_t *)(0x40023800UL + 0x30UL) |= 0x06UL;

//     *(uint32_t *)(0x40020400UL + 0x00UL) |= 0x4000UL;
//     *(uint32_t *)(0x40020400UL + 0x04UL) |= 0x00UL;
//     *(uint32_t *)(0x40020400UL + 0x08UL) |= 0x4000UL;
//     *(uint32_t *)(0x40020400UL + 0x18UL) |= 0x80000UL;
// }

// void GPIO_Init_With_Myself_Macros(void){
//     RCC_AHB1ENR |= RCC_GPIOB_EN | RCC_GPIOC_EN;

//     BIT_SET(GPIOB_MODER, GPIO_PIN_OUT_14);
//     BIT_SET(GPIOB_OTYPER, GPIO_OFF);
//     BIT_SET(GPIOB_OSPEEDR, GPIO_PIN_Med_14);
//     BIT_SET(GPIOB_BSRR, GPIO_PIN_RESET_14);
// }

// void GPIO_Init_CMSIS(void){
//     SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);

//     SET_BIT(GPIOB->MODER, GPIO_MODER_MODER0_0);
//     SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED0_0);
//     SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR0);
// }

// void GPIO_Init_Disco(void)
// {
//     SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN);

//     SET_BIT(GPIOD->MODER, GPIO_MODER_MODE12_0 |
//                               GPIO_MODER_MODE13_0 |
//                               GPIO_MODER_MODE14_0 |
//                               GPIO_MODER_MODE15_0);

//     SET_BIT(GPIOD->BSRR, GPIO_MODER_MODE12_0 |
//                              GPIO_MODER_MODE13_0 |
//                              GPIO_MODER_MODE14_0 |
//                              GPIO_MODER_MODE15_0);

// }