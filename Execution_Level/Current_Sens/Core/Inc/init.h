#pragma once
#ifndef INIT_H
#define INIT_H

#include "../../CMSIS/Devices/STM32F4xx/Inc/stm32f4xx.h"
#include "../../CMSIS/Devices/STM32F4xx/Inc/STM32F429ZI/stm32f429xx.h"

/*Подгруппа без стаорсты 8ТМ51*/

// #define RCC_AHB1ENR             *(uint32_t *)(0x40023800UL + 0x30UL)
// #define RCC_GPIOB_EN            0x02UL
// #define RCC_GPIOC_EN            0x04UL

// #define GPIOB_MODER             *(uint32_t *)(0x40020400UL + 0x00UL)
// #define GPIOB_OTYPER            *(uint32_t *)(0x40020400UL + 0x04UL)
// #define GPIOB_OSPEEDR           *(uint32_t *)(0x40020400UL + 0x08UL)
// #define GPIOB_BSRR              *(uint32_t *)(0x40020400UL + 0x18UL)

// #define GPIOC_IDR               *(uint32_t *)(0x40020800UL + 0x10UL)

// #define GPIO_SPEED_MEDIUM_PIN_7 0x4000UL
// #define GPIO_OUTPUT_MODE_PIN_7  0x4000UL
// #define GPIO_PIN_RESET_7        0x800000UL
// #define GPIO_PIN_SET_7          0x80UL
// #define GPIO_PP_PIN_7           0x80UL
// #define GPIO_PIN_13             0x2000UL

// #define BIT_SET(REG, BIT)       ((REG) |=  (BIT))
// #define BIT_READ(REG, BIT)      ((REG) &   (BIT))
// #define BIT_CLEAR(REG, BIT)     ((REG) &= ~(BIT))

// void GPIO_Init_With_Myself_Macros(void);
// void GPIO_Init_CMSIS(void);

// void GPIO_Init_Memory(void);
// void GPIO_Init_CMSIS(void);
void RCC_Init(void);
void ITR_Init(void);
void GPIO_Init(void);
void SysTick_Init(void);


// volatile uint8_t testik;

#endif


















// #include "../../CMSIS/Devices/STM32F4xx/Inc/stm32f4xx.h"
// #include "../../CMSIS/Devices/STM32F4xx/Inc/STM32F429ZI/stm32f429xx.h"

// // #include <stdint.h>

// #define RCC_AHB1ENR             *(uint32_t *)(0x40023800UL + 0x30UL)
// #define RCC_GPIOB_EN            0x02UL
// #define RCC_GPIOC_EN            0x04UL

// #define GPIOB_MODER             *(uint32_t *)(0x40020400UL + 0x00UL)
// #define GPIOB_OSPEEDR           *(uint32_t *)(0x40020400UL + 0x08UL)
// #define GPIOB_PUPDR             *(uint32_t *)(0x40020400UL + 0x0CUL)
// #define GPIOB_BSRR              *(uint32_t *)(0x40020400UL + 0x18UL)

// #define GPIOC_IDR               *(uint32_t *)(0x40020800UL + 0x10UL)

// #define GPIO_OUTPUT_MODE_PIN_7  0x4000UL
// #define GPIO_SPEED_MED_PIN_7    0x4000UL
// #define GPIO_PIN_RESET_7        0x800000UL
// #define GPIO_PIN_SET_7          0x80UL
// #define GPIO_PIN_13             0x2000UL
// #define GPIO_OFF                0x00UL

// #define BIT_SET(REG, BIT)       ((REG) |= (BIT))
// #define BIT_READ(REG, BIT)      ((REG) & (BIT))

// void GPIO_Init_Memory(void);
// void GPIO_Init_With_Miself_Macros(void);



/* 8Е31 большая подгруппа без старосты*/
// #include "../../CMSIS/Devices/STM32F4xx/Inc/stm32f4xx.h"
// #include "../../CMSIS/Devices/STM32F4xx/Inc/STM32F429ZI/stm32f429xx.h"

// #define RCC_AHB1ENR         *(uint32_t *)(0x40023800UL + 0x30UL)
// #define RCC_GPIOB_EN        0x02UL
// #define RCC_GPIOC_EN        0x04UL

// #define GPIOB_MODER         *(uint32_t *)(0x40020400UL + 0x00UL)
// #define GPIOB_OTYPER        *(uint32_t *)(0x40020400UL + 0x04UL)
// #define GPIOB_OSPEEDR       *(uint32_t *)(0x40020400UL + 0x08UL)
// #define GPIOB_BSRR          *(uint32_t *)(0x40020400UL + 0x18UL)

// #define GPIOC_IDR           *(uint32_t *)(0x40020800UL + 0x10UL)

// #define GPIO_PIN_OUT_7      0x4000UL
// #define GPIO_OFF            0x00UL
// #define GPIO_PIN_Med_7      0x4000UL
// #define GPIO_PIN_RESET_7    0x800000UL
// #define GPIO_PIN_SET_7      0x80UL
// #define GPIO_PIN_13         0x2000UL

// #define BIT_SET(REG, BIT)   ((REG) |= (BIT))
// #define BIT_READ(REG, BIT)  ((REG)  & (BIT))

// void GPIO_Init_Memory(void);
// void GPIO_Init_With_Myself_Macros(void);
// void GPIO_Init_CMSIS(void);

// #include "../../CMSIS/Devices/STM32F4xx/Inc/STM32F411VE/stm32f411xe.h"
// #include "../../CMSIS/Devices/STM32F4xx/Inc/STM32F429ZI/stm32f429xx.h"
// #include "../../CMSIS/Devices/STM32F4xx/Inc/stm32f4xx.h"
// #include <stdint.h>

// #define RCC_AHB1ENR         *(uint32_t *)(0x40023800UL + 0x30UL)
// #define RCC_GPIOB_EN        0x02UL
// #define RCC_GPIOC_EN        0x04UL

// #define GPIOB_MODER         *(uint32_t *)(0x40020400UL + 0x00UL)
// #define GPIOB_OTYPER        *(uint32_t *)(0x40020400UL + 0x04UL)
// #define GPIOB_OSPEEDR       *(uint32_t *)(0x40020400UL + 0x08UL)
// #define GPIOB_BSRR          *(uint32_t *)(0x40020400UL + 0x18UL)

// #define GPIOC_IDR           *(uint32_t *)(0x40020800UL + 0x10UL)

// #define GPIO_PIN_OUT_14     0x10000000UL
// #define GPIO_PIN_Med_14     0x10000000UL
// #define GPIO_PIN_RESET_14   0x40000000UL
// #define GPIO_PIN_SET_14     0x4000UL

// #define GPIO_PIN_OUT_7      0x4000UL
// #define GPIO_PIN_Med_7      0x4000UL
// #define GPIO_OFF            0x00UL
// #define GPIO_PIN_RESET_7    0x800000UL
// #define GPIO_PIN_SET_7      0x80UL
// #define GPIO_PIN_13         0x2000UL

// #define BIT_SET(REG, BIT)   ((REG) |= (BIT))
// #define BIT_READ(REG, BIT)  ((REG) &  (BIT))

// void GPIO_Init_Memory(void);
// void GPIO_Init_With_Myself_Macros(void);
// void GPIO_Init_CMSIS(void);
// void GPIO_Init_CMSIS(void);
// void GPIO_Init_Disco(void);
