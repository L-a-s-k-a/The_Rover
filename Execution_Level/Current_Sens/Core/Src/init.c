#include "init.h"

void RCC_Init(void)
{
    MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U);
    CLEAR_REG(RCC->CFGR);
    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET);
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET);
    CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON);
    while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET);
    CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);

    SET_BIT(RCC->CR, RCC_CR_HSEON); // Включение внешнего источника тактирования высокой частоты
    while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET);
    SET_BIT(RCC->CR, RCC_CR_CSSON); // Включение Clock Security

    /*------------------Настройка на 168 МГц------------------*/
    CLEAR_REG(RCC->PLLCFGR);
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_HSE); // Источник тактирвоания HSE
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_2);     // Деление источника тактирования на 4 (PLLM)
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_2 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_7); // Настройка умножения на 84 (PLLN) (0101 0100)
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP);     // Деление частоты после умножения на 2 (PLLP)
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1); // Деление частоты после умножения на 3 (PLLQ)

    SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);                         // В качестве системного тактирования выбран PLL
    SET_BIT(RCC->CFGR, RCC_CFGR_HPRE_DIV1);                      // Предделитель шины AHB1 настроен на 1 (без деления)
    SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV4);                     // Предделитель шины APB1 настроен на 4 (42МГц)
    SET_BIT(RCC->CFGR, RCC_CFGR_PPRE2_DIV2);                     // Предделитель шины APB2 настроен на 2 (84МГц)
    SET_BIT(RCC->CFGR, RCC_CFGR_MCO1);                           // Нстройка вывода частоты PLL на MCO1
    SET_BIT(RCC->CFGR, RCC_CFGR_MCO1PRE_2 | RCC_CFGR_MCO1PRE_1); // Предделитель 4 для вывода на MCO1
    SET_BIT(RCC->CFGR, RCC_CFGR_MCO2_1);                         // Настройка вывода частоты SYSCLK на MCO2
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
    MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, (168000 - 1) << SysTick_LOAD_RELOAD_Pos);
    MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, (168000 - 1) << SysTick_VAL_CURRENT_Pos);
    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
}

void GPIO_Init(void)
{
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN);

    SET_BIT(GPIOA->MODER, GPIO_MODER_MODER7_1);
    // SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR7);
    SET_BIT(GPIOA->AFR[0], GPIO_AFRL_AFSEL7_1);

    SET_BIT(GPIOA->MODER, GPIO_MODER_MODER6_1);
    // SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR6);
    SET_BIT(GPIOA->AFR[0], GPIO_AFRL_AFSEL6_1);

    /*--------Настройка работы порта PC9 в качестве MCO2--------*/
    SET_BIT(GPIOC->MODER, GPIO_MODER_MODER9_1);
    SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED9);
    CLEAR_BIT(GPIOC->AFR[1], GPIO_AFRH_AFSEL9);

    /*--------Настройка работы порта PA8 в качестве MCO1--------*/
    SET_BIT(GPIOA->MODER, GPIO_MODER_MODER8_1);
    SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED8);
    CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL8);
}

void TIM_PWM_Init(void){
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN); //Включение тактирования таймера 3
    TIM3->PSC = 100-1; //42 МГц тактовая частота шины АРВ1, но таймеры тактируются от источника, который всегда умножает на 2
    TIM3->ARR = 8400-1;

    /*--------------Режим переключения--------------*/
    CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_CC1S_Msk);
    // SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1); // OC1REF Toggles when TIM3_CNT=TIM3_CCR1.
    SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE);
    SET_BIT(TIM3->CCER, TIM_CCER_CC1E); 
    TIM3->CCR1 = 8050-1; 

    /*------------------Режим ШИМ------------------*/
    CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_CC2S_Msk);
    SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE); //  In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1 else inactive.
    SET_BIT(TIM3->CCER, TIM_CCER_CC2E);
    TIM3->CCR2 = 3150-1; 

    SET_BIT(TIM3->CR2, TIM_CR2_MMS_1); 
    SET_BIT(TIM3->CR1, TIM_CR1_CEN); //Включение 
    SET_BIT(TIM3->DIER, TIM_DIER_UIE); //Включение Прерывания по переполнению
    NVIC_EnableIRQ(TIM3_IRQn);
}

void ADC_Init_Polling(void){
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);

    //Натсройка канала PA0 для работы в аналоговом режиме
    SET_BIT(GPIOA->MODER, GPIO_MODER_MODER0);
    CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD0);
    SET_BIT(GPIOA->MODER, GPIO_MODER_MODER1);
    CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD1);
    SET_BIT(GPIOA->MODER, GPIO_MODER_MODER2);
    CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD2);

    /* Установка предделителя на 4, АЦП тактируется от шины АРВ2 84МГц, 
     * а АЦП должно работать на частоте не более 30 МГц, из-за чего
     * следует устанавливать делитель на 4
     */
    SET_BIT(ADC->CCR, ADC_CCR_ADCPRE_0);

    //Включение внутреннего датчика температуры
    SET_BIT(ADC->CCR, ADC_CCR_TSVREFE);

    CLEAR_BIT(ADC1->CR1, ADC_CR1_RES);
    CLEAR_BIT(ADC1->CR1, ADC_CR1_SCAN);
    SET_BIT(ADC1->CR1, ADC_CR1_EOCIE);

    CLEAR_BIT(ADC1->CR2, ADC_CR2_ALIGN);
    CLEAR_BIT(ADC1->CR2, ADC_CR2_CONT);

    // SET_BIT(ADC1->CR1, ADC_CR1_JAUTO);
    // SET_BIT(ADC1->CR2, ADC_CR2_JEXTEN_0); //Разрешение запуска преобразований по внешнему триггеру
    // SET_BIT(ADC1->CR2, ADC_CR2_JEXTSEL_2); //Запуск по CC2 3-го таймера

    // SET_BIT(ADC1->CR2, ADC_CR2_EXTEN_0);
    // SET_BIT(ADC1->CR2, ADC_CR2_EXTSEL_3);

    //Количество сэмплов преобраования на 15
    SET_BIT(ADC1->SMPR2, ADC_SMPR2_SMP0_0); //Для канала 0
    SET_BIT(ADC1->SMPR2, ADC_SMPR2_SMP1_0); // Для канала 1
    SET_BIT(ADC1->SMPR1, ADC_SMPR1_SMP16_0); //Для канала 16

    //Выбор канала и последовательности считывания
    CLEAR_BIT(ADC1->SQR1, ADC_SQR1_L); //Общее число преобразований - 3
    CLEAR_BIT(ADC1->SQR3, ADC_SQR3_SQ1); //1 channel

    // CLEAR_BIT(ADC1->JSQR, ADC_JSQR_JL);
    // SET_BIT(ADC1->JSQR, ADC_JSQR_JSQ4_0);

    //Включение АЦП
    SET_BIT(ADC1->CR2, ADC_CR2_ADON);
    NVIC_EnableIRQ(ADC_IRQn);
}

void TIM_ENCODER_Init(void){
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN); //Включение тактирования таймера 2
    SET_BIT(TIM2->CCMR1, TIM_CCMR1_CC1S_0); //Настройка выхода CC1 (Capture/Compare 1) на вход, IC1(Input capture 1) используется как TI1
    SET_BIT(TIM2->CCMR1, TIM_CCMR1_CC2S_0); //Настройка выхода CC2 (Capture/Compare 1) на вход, IC2(Input capture 1) используется как TI2
    /*
     *Настройка режима подчинения (Slave Mode Selection). 
     *Выбран режим энкодерного интерфейса.
     *Счётчик считает ввех/вниз по фронту сигнала на линии 1 - TI1FP1 (CH1-PA5)
     *в зависимости от уровня сигнала на линии 2 - TI2FP2. (CH2-PB3)
     */
    SET_BIT(TIM2->SMCR, TIM_SMCR_SMS_0); 
    SET_BIT(TIM2->CCMR1, TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_3); //Установка количества пропускаемых сэмплов N = 8
    SET_BIT(TIM2->CCMR1, TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_IC2F_2 | TIM_CCMR1_IC2F_3); //Установка количества пропускаемых сэмплов N = 8
    
    TIM2->ARR = 6000 - 1;
    CLEAR_BIT(TIM2->CCER, TIM_CCER_CC1P);
    SET_BIT(TIM2->CCER, TIM_CCER_CC1E); 
    SET_BIT(TIM2->DIER, TIM_DIER_UIE); //Включение Прерывания по переполнению
    // Enc_Trig_Init();
    
    SET_BIT(TIM2->CR1, TIM_CR1_CEN); //Включение счётчика
    NVIC_EnableIRQ(TIM2_IRQn);
}

void Enc_Trig_Init(void){
    /* 0000: No filter, sampling is done at fDTS */
    CLEAR_BIT(TIM2->SMCR, TIM_SMCR_ETF);

    /* 100: TI1 Edge Detector (TI1F_ED) */
    CLEAR_BIT(TIM2->SMCR, TIM_SMCR_TS_0 | TIM_SMCR_TS_1);
    SET_BIT(TIM2->SMCR, TIM_SMCR_TS_2);

    /* 1: Trigger interrupt enabled */
    SET_BIT(TIM2->DIER, TIM_DIER_TIE); //Включение Прерывания по переполнению

    NVIC_EnableIRQ(TIM2_IRQn);
}